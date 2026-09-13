/*
 * SPDX-FileCopyrightText: Brian Kuschak <bkuschak@gmail.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * TCP server for the CMSIS-DAP TCP protocol as used by OpenOCD. CMSIS-DAP
 * requests are variable length; USB preserves transfer sizes but TCP does not,
 * so each request and response is framed with the short header below.
 *
 * Derived from main/cmsis_dap_tcp.c of
 * https://github.com/bkuschak/cmsis_dap_tcp_esp32/, which implements the same
 * protocol on an ESP32. The wire format (packet header, signature, framing) is
 * kept byte-for-byte identical so the two speak to the same OpenOCD backend.
 * The port drops the FreeRTOS multi-task structure -- one Linux process serves
 * one interface -- and swaps ESP-IDF logging for logging.h.
 */

#include "tcp_server.h"

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>

#include <signal.h>

#include <arpa/inet.h>
#include <netdb.h>
#include <netinet/tcp.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/un.h>

#include <algorithm>
#include <chrono>
#include <csignal>
#include <thread>

#include "DAP.h"
#include "logging.h"

namespace tcp_server {

namespace {

// Set by the SIGINT/SIGTERM handler; polled by the accept/select loops so
// the process can release the GPIO pins and exit cleanly instead of being
// killed mid-transfer.
volatile std::sig_atomic_t g_should_exit = 0;

// SIGINT/SIGTERM stay blocked everywhere except inside the pselect() below,
// which unblocks them for exactly as long as it waits. Without that, a signal
// arriving between the loop's g_should_exit check and the start of the wait
// would set the flag with nobody left to look at it, and the wait -- which has
// no timeout -- would block until a client happened to send something.
sigset_t g_exit_signals;    // SIGINT | SIGTERM
sigset_t g_wait_mask;       // what to unblock while waiting: nothing else

void handle_exit_signal(int)
{
    g_should_exit = 1;
}

// Wire format is little endian. These are typed functions rather than macros so
// the conversion cannot silently widen or truncate its argument.
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
constexpr uint16_t bswap_if_be_u16(uint16_t v) { return v; }
constexpr uint32_t bswap_if_be_u32(uint32_t v) { return v; }
#else
#include <byteswap.h>
constexpr uint16_t bswap_if_be_u16(uint16_t v) { return __bswap_16(v); }
constexpr uint32_t bswap_if_be_u32(uint32_t v) { return __bswap_32(v); }
#endif

constexpr uint16_t le_to_h_u16(uint16_t v) { return bswap_if_be_u16(v); }
constexpr uint32_t le_to_h_u32(uint32_t v) { return bswap_if_be_u32(v); }
constexpr uint16_t h_u16_to_le(uint16_t v) { return bswap_if_be_u16(v); }
constexpr uint32_t h_u32_to_le(uint32_t v) { return bswap_if_be_u32(v); }

// CMSIS-DAP maximum packet size, which must be >= what the client (OpenOCD)
// uses. Taken from the DAP command handlers so the two cannot drift apart.
constexpr size_t DAP_PKT_SIZE = cmsis::dap::MAX_PACKET_SIZE;

constexpr uint32_t DAP_PKT_HDR_SIGNATURE = 0x00504144;   // "DAP\0" in LE
constexpr uint8_t DAP_PKT_TYPE_REQUEST  = 0x01;
constexpr uint8_t DAP_PKT_TYPE_RESPONSE = 0x02;

// CMSIS-DAP requests are variable length. With CMSIS-DAP over USB, the
// transfer sizes are preserved by the USB stack. However, TCP/IP is stream
// oriented so we perform our own packetization to preserve the boundaries
// between each request. This short header is prepended to each CMSIS-DAP
// request and response before being sent over the socket. Little endian format
// is used for multibyte values.
struct cmsis_dap_tcp_packet_hdr {
    uint32_t signature;         // "DAP"
    uint16_t length;            // Not including header length.
    uint8_t packet_type;
    uint8_t reserved;           // Reserved for future use.
} __attribute__((__packed__));

// The packed struct above is the wire format, parsed byte-for-byte by the peer,
// so its size is part of the protocol rather than an implementation detail.
static_assert(sizeof(struct cmsis_dap_tcp_packet_hdr) == 8,
              "cmsis_dap_tcp_packet_hdr must stay 8 bytes on the wire");

// One framed packet: header plus a maximum-size CMSIS-DAP payload.
constexpr size_t DAP_TOTAL_PKT_SIZE =
    sizeof(struct cmsis_dap_tcp_packet_hdr) + DAP_PKT_SIZE;

// Enough room for everything we told the client it may have in flight at once
// (DAP_Info's packet count), so a client that takes us at our word and
// pipelines that many requests is drained in one pass instead of costing an
// extra recv() round trip per packet.
constexpr size_t MSGBUF_SIZE = cmsis::dap::MAX_PACKET_COUNT * DAP_TOTAL_PKT_SIZE;

static_assert(MSGBUF_SIZE > DAP_TOTAL_PKT_SIZE,
              "receive buffer must hold more than one maximum-size packet");

struct msgbuf_t {
    uint8_t  data[MSGBUF_SIZE];
    size_t   len;
};

// Sized for the worst-case response amplification rather than for DAP_PKT_SIZE:
// the DAP command handlers do not bounds-check their own output, so a request
// that legitimately expands past the packet size (see
// MAX_RESPONSE_AMPLIFICATION) would otherwise run off the end of this buffer
// before send_dap_response() ever got to reject it.
constexpr size_t RESPONSE_BUF_SIZE =
    DAP_PKT_SIZE * cmsis::dap::MAX_RESPONSE_AMPLIFICATION;

uint8_t response[RESPONSE_BUF_SIZE];
uint8_t packet_buf[DAP_TOTAL_PKT_SIZE];

// ---------------------------------------------------------------------------
// Use our own receive buffer to accumulate from the socket until a complete
// message packet is available.

void msgbuf_init(struct msgbuf_t *buf)
{
    buf->len = 0;
}

// Read all data from the socket into our buffer.
int msgbuf_add(struct msgbuf_t *buf, int sock)
{
    size_t space = sizeof(buf->data) - buf->len;
    if (space == 0)
        return -ENOSPC;

    ssize_t n = recv(sock, buf->data + buf->len, space, 0);
    if (n < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK)
            return 0;       // no new data
        LOGE_KV("socket read error", "err=\"%s\"", strerror(errno));
        return -errno;
    }
    if (n == 0) {
        return -ENOTCONN;   // connection closed
    }
    buf->len += (size_t)n;
    return 0;
}

// Read a complete CMSIS-DAP request packet from our buffer.
// After done using it, call msgbuf_consume(buf, total_len).
int msgbuf_parse(struct msgbuf_t *buf,
        struct cmsis_dap_tcp_packet_hdr *hdr, const uint8_t **payload,
        size_t *payload_len, size_t *total_len)
{
    if (buf->len < sizeof(struct cmsis_dap_tcp_packet_hdr))
        return -EAGAIN;

    struct cmsis_dap_tcp_packet_hdr tmp;
    memcpy(&tmp, buf->data, sizeof(tmp));
    tmp.signature = le_to_h_u32(tmp.signature);
    tmp.length = le_to_h_u16(tmp.length);

    if (tmp.signature != DAP_PKT_HDR_SIGNATURE) {
        LOGW_KV("invalid header signature",
                "signature=0x%08lx", (unsigned long)tmp.signature);
        return -EINVAL;
    }

    if (tmp.packet_type != DAP_PKT_TYPE_REQUEST) {
        LOGW_KV("unrecognized packet type", "type=0x%02x", tmp.packet_type);
        return -EINVAL;
    }

    // The length field is 16 bits, so it can claim far more than a CMSIS-DAP
    // request may contain. Reject that here rather than handing an oversized
    // request to command handlers written on the assumption that the transport
    // already enforced this limit. It also keeps the wait below bounded: a
    // length this buffer can never hold would otherwise stall the connection
    // until the buffer filled.
    if (tmp.length > DAP_PKT_SIZE) {
        LOGW_KV("request longer than the maximum packet size",
                "len=%u max=%zu", (unsigned)tmp.length, DAP_PKT_SIZE);
        return -EINVAL;
    }

    if (buf->len < sizeof(*hdr) + tmp.length)
        return -EAGAIN;

    // A complete packet is available.
    *hdr = tmp;
    *payload = buf->data + sizeof(*hdr);
    if(payload_len) *payload_len = tmp.length;
    if(total_len) *total_len = tmp.length + sizeof(*hdr);
    LOGD_KV("received packet", "len=%d", hdr->length);

    return 0;
}

// Discard data from the buffer.
void msgbuf_consume(struct msgbuf_t *buf, size_t n)
{
    if(n > buf->len) n = buf->len;
    memmove(buf->data, buf->data + n, buf->len - n);
    buf->len -= n;

}

// ---------------------------------------------------------------------------

int send_dap_response(int sock, const uint8_t *payload, uint16_t len)
{
    if (len > DAP_PKT_SIZE) {
        errno = EMSGSIZE;
        LOGE_KV("response too large for buffer", "len=%u max=%zu", (unsigned)len, DAP_PKT_SIZE);
        return -1;
    }

    struct cmsis_dap_tcp_packet_hdr hdr;
    hdr.signature = h_u32_to_le(DAP_PKT_HDR_SIGNATURE);
    hdr.length = h_u16_to_le(len);
    hdr.packet_type = DAP_PKT_TYPE_RESPONSE;
    hdr.reserved = 0;

    memcpy(packet_buf, &hdr, sizeof(hdr));
    memcpy(packet_buf + sizeof(hdr), payload, len);

    size_t total_len = sizeof(hdr) + len;
    size_t sent = 0;

    while (sent < total_len) {
        ssize_t n = write(sock, packet_buf + sent, total_len - sent);
        if (n < 0) {
            if (errno == EINTR) {
                continue;   // retry
            }
            else if (errno == EAGAIN || errno == EWOULDBLOCK) {
                // Socket is non-blocking; wait until writable rather than
                // dropping the client on a momentarily full send buffer.
                fd_set write_fds;
                FD_ZERO(&write_fds);
                FD_SET(sock, &write_fds);
                struct timeval tv;
                tv.tv_sec = 5;
                tv.tv_usec = 0;
                int sel = select(sock + 1, NULL, &write_fds, NULL, &tv);
                if (sel <= 0) {
                    LOGE("socket write timed out, dropping client");
                    return -1;
                }
                continue;   // retry write
            }
            else {
                LOGE_KV("socket write error", "err=\"%s\"", strerror(errno));
                return -1;
            }
        }
        sent += (size_t)n;
    }

    return 0;
}

int process_dap_request(int sock, const uint8_t *request, uint16_t len)
{
    (void)len;

    // DAP_ExecuteCommand returns:
    //   number of bytes in response (lower 16 bits)
    //   number of bytes in request (upper 16 bits)
    int ret = cmsis::dap::execute_cmd(request, response);

    int request_len = (ret>>16) & 0xFFFF;
    int response_len = ret & 0xFFFF;
    LOGD_KV("processed command",
            "request_len=%d response_len=%d", request_len, response_len);

    return send_dap_response(sock, response, response_len);
}

void set_nonblocking(int fd)
{
    int flags = fcntl(fd, F_GETFL, 0);
    if (flags < 0) flags = 0;
    fcntl(fd, F_SETFL, flags | O_NONBLOCK);
}

// CMSIS-DAP over TCP is strictly request/response: nothing else is queued
// behind a response that Nagle could usefully coalesce it with, and waiting
// for the peer's ACK before sending a small one costs a delayed-ACK stall on
// every single DAP command.
void set_nodelay(int fd)
{
    int val = 1;
    if (setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &val, sizeof(val)) < 0) {
        LOGW_KV("failed to set TCP_NODELAY", "err=\"%s\"", strerror(errno));
    }
}

// TCP keepalives detect a client that vanished without closing the socket,
// which would otherwise hold the single-client slot forever.
constexpr bool DAP_TCP_USE_KEEPALIVE = true;
constexpr int DAP_TCP_KEEPALIVE_TIMEOUT = 10;

void set_keepalives(int fd)
{
    if (!DAP_TCP_USE_KEEPALIVE) {
        return;
    }

    int val = 1;
    setsockopt(fd, SOL_SOCKET, SO_KEEPALIVE, &val, sizeof(val));

    // Seconds between probes.
    val = 1;
    setsockopt(fd, IPPROTO_TCP, TCP_KEEPIDLE, &val, sizeof(val));
    setsockopt(fd, IPPROTO_TCP, TCP_KEEPINTVL, &val, sizeof(val));

    // Number of probes to send before closing the connection.
    val = DAP_TCP_KEEPALIVE_TIMEOUT;
    setsockopt(fd, IPPROTO_TCP, TCP_KEEPCNT, &val, sizeof(val));

    LOGD_KV("using tcp keepalives", "timeout_s=%d", DAP_TCP_KEEPALIVE_TIMEOUT);
}


// Create, bind (per bind_addr / port) and listen on the server socket.
// Returns the listening fd, or -1 on failure (errno / a log line explain why).
int create_listener(const char *bind_addr, int port)
{
    int listener_fd = -1;

    if (bind_addr == NULL || *bind_addr == '\0') {
        // Default: dual-stack wildcard, accepts both IPv4 and IPv6 clients.
        struct sockaddr_in6 addr;
        memset(&addr, 0, sizeof(addr));
        addr.sin6_family = AF_INET6;
        addr.sin6_addr = in6addr_any;
        addr.sin6_port = htons(port);

        listener_fd = socket(AF_INET6, SOCK_STREAM, 0);
        if (listener_fd < 0) {
            LOGE_KV("failed to create listening socket", "err=\"%s\"", strerror(errno));
            return -1;
        }

        int no = 0;
        if (setsockopt(listener_fd, IPPROTO_IPV6, IPV6_V6ONLY, &no, sizeof(no)) < 0) {
            LOGE_KV("failed to disable IPV6_V6ONLY", "err=\"%s\"", strerror(errno));
        }

        int yes = 1;
        setsockopt(listener_fd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));

        if (bind(listener_fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
            LOGE_KV("bind failed", "addr=[::] port=%d err=\"%s\"", port, strerror(errno));
            close(listener_fd);
            return -1;
        }
    } else {
        // A specific bind address was requested; resolve it and try each
        // candidate (covers both IPv4 and IPv6 literals) until one binds.
        struct addrinfo hints;
        memset(&hints, 0, sizeof(hints));
        hints.ai_family = AF_UNSPEC;
        hints.ai_socktype = SOCK_STREAM;
        hints.ai_flags = AI_PASSIVE | AI_NUMERICHOST;

        char port_str[16];
        snprintf(port_str, sizeof(port_str), "%d", port);

        struct addrinfo *results = NULL;
        int gai_ret = getaddrinfo(bind_addr, port_str, &hints, &results);
        if (gai_ret != 0) {
            LOGE_KV("failed to resolve bind address",
                    "addr=%s err=\"%s\"", bind_addr, gai_strerror(gai_ret));
            return -1;
        }

        for (struct addrinfo *ai = results; ai != NULL; ai = ai->ai_next) {
            listener_fd = socket(ai->ai_family, ai->ai_socktype, ai->ai_protocol);
            if (listener_fd < 0) {
                continue;
            }

            int yes = 1;
            setsockopt(listener_fd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));

            if (bind(listener_fd, ai->ai_addr, ai->ai_addrlen) == 0) {
                break;      // success
            }

            close(listener_fd);
            listener_fd = -1;
        }
        freeaddrinfo(results);

        if (listener_fd < 0) {
            LOGE_KV("bind failed", "addr=%s port=%d", bind_addr, port);
            return -1;
        }
    }

    if (listen(listener_fd, 1) < 0) {
        LOGE_KV("failed to listen on socket", "err=\"%s\"", strerror(errno));
        close(listener_fd);
        return -1;
    }

    set_nonblocking(listener_fd);
    return listener_fd;
}

// Create, bind and listen on a Unix domain socket at `path`, for a client on
// the same machine as the gateway. Skips the TCP/IP stack and the two
// TCP_NODELAY/keepalive setsockopt calls entirely (neither means anything on
// AF_UNIX), which is the point: one less network round trip's worth of
// kernel work on a path that was already local. Everything past accept() --
// framing, DAP dispatch, GPIO/SWD -- is identical to the TCP listener; only
// the transport differs.
int create_unix_listener(const char *path)
{
    // A stale socket file from an unclean shutdown would otherwise fail the
    // bind with EADDRINUSE forever.
    unlink(path);

    int listener_fd = socket(AF_UNIX, SOCK_STREAM, 0);
    if (listener_fd < 0) {
        LOGE_KV("failed to create unix listening socket", "err=\"%s\"", strerror(errno));
        return -1;
    }

    struct sockaddr_un addr;
    memset(&addr, 0, sizeof(addr));
    addr.sun_family = AF_UNIX;
    if (strlen(path) >= sizeof(addr.sun_path)) {
        LOGE_KV("unix socket path too long",
                "path=%s max=%zu", path, sizeof(addr.sun_path) - 1);
        close(listener_fd);
        return -1;
    }
    strncpy(addr.sun_path, path, sizeof(addr.sun_path) - 1);

    if (bind(listener_fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        LOGE_KV("bind failed", "path=%s err=\"%s\"", path, strerror(errno));
        close(listener_fd);
        return -1;
    }

    if (listen(listener_fd, 1) < 0) {
        LOGE_KV("failed to listen on unix socket", "err=\"%s\"", strerror(errno));
        close(listener_fd);
        unlink(path);
        return -1;
    }

    set_nonblocking(listener_fd);
    return listener_fd;
}

}  // namespace

void install_exit_handlers(void)
{
    std::signal(SIGINT, handle_exit_signal);
    std::signal(SIGTERM, handle_exit_signal);

    sigemptyset(&g_exit_signals);
    sigaddset(&g_exit_signals, SIGINT);
    sigaddset(&g_exit_signals, SIGTERM);
    sigemptyset(&g_wait_mask);

    // Blocked from here on, and unblocked only by pselect(). A signal that
    // arrives while a transfer is in progress therefore stays pending until
    // the server is next idle, which is what we want: it means shutdown never
    // interrupts a half-finished SWD transaction.
    sigprocmask(SIG_BLOCK, &g_exit_signals, NULL);
}

void serve(const char *bind_addr, int port, const char *unix_socket_path)
{
    const bool use_unix = (unix_socket_path != NULL) && (*unix_socket_path != '\0');
    struct msgbuf_t buf;

    while (!g_should_exit) {
        int listener_fd = use_unix ? create_unix_listener(unix_socket_path)
                                   : create_listener(bind_addr, port);
        if (listener_fd < 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            continue;
        }

        if (use_unix) {
            LOGI_KV("listening", "unix_socket=%s", unix_socket_path);
        } else {
            LOGI_KV("listening", "addr=%s port=%d",
                    (bind_addr == NULL || *bind_addr == '\0') ? "[::]" : bind_addr, port);
        }

        msgbuf_init(&buf);

        // Only one active client at a time is allowed.
        int client_fd = -1;
        int run = 0;

        while (!g_should_exit) {
            struct sockaddr_storage client_addr;
            socklen_t addr_len = sizeof(client_addr);
            char ipstr[INET6_ADDRSTRLEN];

            fd_set read_fds;
            FD_ZERO(&read_fds);
            FD_SET(listener_fd, &read_fds);
            if (client_fd >= 0)
                FD_SET(client_fd, &read_fds);

            int fdmax = std::max(client_fd, listener_fd);

            int sel = pselect(fdmax + 1, &read_fds, NULL, NULL, NULL, &g_wait_mask);
            if (sel < 0) {
                if (errno == EINTR)
                    continue;   // signal delivered; loop re-checks g_should_exit
                LOGE_KV("select error", "err=\"%s\"", strerror(errno));
                break;  // retry listener
            }

            LOGD_KV("select loop iteration", "run=%d", ++run);

            // New connection?
            if (FD_ISSET(listener_fd, &read_fds)) {
                int new_fd = accept(listener_fd,
                                    (struct sockaddr *)&client_addr,
                                    &addr_len);
                if (new_fd < 0) {
                    if (errno != EAGAIN && errno != EWOULDBLOCK) {
                        // Just ignore error for now.
                        LOGE_KV("accept error", "err=\"%s\"", strerror(errno));
                    }
                } else {
                    if (client_fd >= 0) {
                        LOGW("rejecting connection, another client is already connected");
                        // Force an abortive close (RST) instead of a graceful FIN, so the
                        // rejected client sees an unambiguous connection-reset error
                        // instead of a silent empty read that looks like the server just
                        // hung up for no reason.
                        struct linger lg;
                        lg.l_onoff = 1;
                        lg.l_linger = 0;
                        setsockopt(new_fd, SOL_SOCKET, SO_LINGER, &lg, sizeof(lg));
                        close(new_fd);
                        continue;   // restart select() loop
                    }

                    int client_port = 0;
                    ipstr[0] = '\0';
                    if (client_addr.ss_family == AF_INET) {
                        // IPv4
                        struct sockaddr_in *s =
                            (struct sockaddr_in *)&client_addr;
                        inet_ntop(AF_INET, &s->sin_addr,
                                  ipstr, sizeof(ipstr));
                        client_port = ntohs(s->sin_port);
                    } else if (client_addr.ss_family == AF_INET6) {
                        // IPv6
                        struct sockaddr_in6 *s =
                            (struct sockaddr_in6 *)&client_addr;
                        inet_ntop(AF_INET6, &s->sin6_addr,
                                  ipstr, sizeof(ipstr));
                        client_port = ntohs(s->sin6_port);
                    }
                    if (use_unix) {
                        LOGI("client connected (unix socket)");
                    } else {
                        LOGI_KV("client connected", "addr=%s port=%d", ipstr, client_port);
                    }
                    set_nonblocking(new_fd);
                    if (!use_unix) {
                        // Neither option means anything on AF_UNIX: there is
                        // no Nagle algorithm and no TCP keepalive timer to
                        // configure on a socket that never touches the
                        // network stack.
                        set_nodelay(new_fd);
                        set_keepalives(new_fd);
                    }
                    client_fd = new_fd;
                    msgbuf_init(&buf);
                    continue;   // restart select() loop
                }
            }

            // Data from client?
            if (client_fd >= 0 && FD_ISSET(client_fd, &read_fds)) {
                int add_ret = msgbuf_add(&buf, client_fd);
                if (add_ret < 0) {
                    if (add_ret == -ENOSPC) {
                        LOGW("receive buffer full (corrupt stream?), disconnecting");
                    } else {
                        LOGI("client disconnected");
                    }
                    close(client_fd);
                    client_fd = -1;
                    continue;   // restart select() loop
                }

                // Process all the DAP requests in our buffer.
                struct cmsis_dap_tcp_packet_hdr hdr;
                const uint8_t *payload;
                while (true) {
                    size_t payload_len;
                    size_t total_len;
                    int ret = msgbuf_parse(&buf, &hdr, &payload,
                                           &payload_len, &total_len);
                    if (ret == -EAGAIN) {
                        break;      // incomplete packet, wait for more data
                    }
                    if (ret < 0) {
                        // Unrecoverable framing error (bad signature/type). There is
                        // no way to resynchronize with the stream, so drop the client.
                        LOGW("framing error, disconnecting");
                        close(client_fd);
                        client_fd = -1;
                        break;
                    }

                    ret = process_dap_request(client_fd,
                                              payload,
                                              (uint16_t)payload_len);
                    msgbuf_consume(&buf, total_len);

                    // If we cannot process the request and response, just close
                    // the connection.
                    if (ret < 0) {
                        LOGI("disconnecting");
                        close(client_fd);
                        client_fd = -1;
                        break;
                    }
                }
            }
        }

        if (client_fd >= 0)
            close(client_fd);
        close(listener_fd);
        if (use_unix) {
            unlink(unix_socket_path);
        }

        if (g_should_exit) {
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));  // retry listener
    }
}

}  // namespace tcp_server
