/*
 * SPDX-FileCopyrightText: Brian Kuschak <bkuschak@gmail.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * TCP server for the CMSIS-DAP TCP protocol as used by OpenOCD.
 *
 * Derived from main/cmsis_dap_tcp.h of
 * https://github.com/bkuschak/cmsis_dap_tcp_esp32/ (the ESP32 implementation
 * of the same protocol), adapted from FreeRTOS tasks to a single Linux
 * process.
 */

#ifndef TCP_SERVER_H
#define TCP_SERVER_H

namespace tcp_server {

// Catch SIGINT/SIGTERM so serve() can return instead of the process being
// killed mid-transfer, leaving the GPIO pins driven.
void install_exit_handlers(void);

// Serve CMSIS-DAP requests until one of those signals arrives, on a Unix
// domain socket at unix_socket_path if it is non-NULL and non-empty
// (bind_addr/port are then ignored), otherwise on bind_addr:port. bind_addr
// may be NULL or empty for the dual-stack wildcard. Both pointers must
// outlive the call. Returns only on exit; a listener that cannot be created
// is retried once a second.
//
// The Unix socket is for a client on the same machine: it skips the TCP/IP
// stack entirely, which is one less network round trip's worth of kernel
// work per request -- measurable in a tight benchmarking loop, irrelevant
// for driving OpenOCD, which needs TCP to reach the gateway at all.
void serve(const char *bind_addr, int port, const char *unix_socket_path);

}  // namespace tcp_server

#endif  // TCP_SERVER_H
