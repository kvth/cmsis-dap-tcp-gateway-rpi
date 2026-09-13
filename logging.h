#ifndef LOGGING_H
#define LOGGING_H

// logfmt-style logging: one record per line, machine-readable and greppable,
//
//   level=info msg="client connected" addr=127.0.0.1 port=4441
//
// The message is a plain string, quoted only when it contains something that
// would break the format; any values belong in key=val fields after it.
//
// Under systemd, records are additionally prefixed with the "<N>" syslog
// priority that journald parses off the stream, so `journalctl -p warning`
// and friends filter correctly. The prefix is only emitted when the stream is
// actually a journal stream, so it never shows up in a terminal or a pipe.

namespace logging {

// Ordered by severity: filtering keeps everything up to and including the
// configured level.
enum class Level {
    error = 0,
    warn  = 1,
    info  = 2,
    debug = 3,
};

// Parse a --log-level value ("error", "warn", "info", "debug"). Returns false
// and leaves *out alone if the name is not one of those.
bool parse_level(const char *name, Level *out);
const char *level_name(Level level);

// Set the level records are filtered against, and work out whether stdout and
// stderr are journal streams. Call once at startup, before anything logs.
void init(Level level);

// Whether a record at this level would be emitted. The macros below check this
// before formatting anything.
bool enabled(Level level);

// Emit one record. write_kv()'s `fields` is a printf format producing
// "key=val key=val"; quoting values that need it is the call site's job.
void write(Level level, const char *msg);
void write_kv(Level level, const char *msg, const char *fields, ...)
    __attribute__((format(printf, 3, 4)));

}  // namespace logging

#define LOG_IF(level, msg) \
    do { \
        if (::logging::enabled(level)) { \
            ::logging::write(level, msg); \
        } \
    } while (0)

#define LOG_IF_KV(level, msg, fields, ...) \
    do { \
        if (::logging::enabled(level)) { \
            ::logging::write_kv(level, msg, fields, ##__VA_ARGS__); \
        } \
    } while (0)

#define LOGE(msg) LOG_IF(::logging::Level::error, msg)
#define LOGW(msg) LOG_IF(::logging::Level::warn,  msg)
#define LOGI(msg) LOG_IF(::logging::Level::info,  msg)
#define LOGD(msg) LOG_IF(::logging::Level::debug, msg)

#define LOGE_KV(msg, fields, ...) \
    LOG_IF_KV(::logging::Level::error, msg, fields, ##__VA_ARGS__)
#define LOGW_KV(msg, fields, ...) \
    LOG_IF_KV(::logging::Level::warn,  msg, fields, ##__VA_ARGS__)
#define LOGI_KV(msg, fields, ...) \
    LOG_IF_KV(::logging::Level::info,  msg, fields, ##__VA_ARGS__)
#define LOGD_KV(msg, fields, ...) \
    LOG_IF_KV(::logging::Level::debug, msg, fields, ##__VA_ARGS__)

#endif  // LOGGING_H
