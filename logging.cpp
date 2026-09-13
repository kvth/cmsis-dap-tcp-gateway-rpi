#include "logging.h"

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <sys/stat.h>

namespace logging {

namespace {

Level g_level = Level::info;

// Whether the respective stream is the one systemd handed us, in which case a
// "<N>" priority prefix is picked up by journald instead of being displayed.
bool g_journal_stdout = false;
bool g_journal_stderr = false;

struct LevelInfo {
    const char *name;
    const char *journal_prefix;   // syslog priority, as journald expects it
};

// Indexed by Level.
constexpr LevelInfo LEVELS[] = {
    { "error", "<3>" },
    { "warn",  "<4>" },
    { "info",  "<6>" },
    { "debug", "<7>" },
};

const LevelInfo &info_for(Level level)
{
    return LEVELS[static_cast<int>(level)];
}

// systemd sets JOURNAL_STREAM to the "device:inode" of the stream it connected
// to the journal; a service whose stdout/stderr goes anywhere else (a terminal,
// a pipe, a file) either has no such variable or one that does not match.
bool is_journal_stream(int fd)
{
    const char *env = getenv("JOURNAL_STREAM");
    if (env == NULL) {
        return false;
    }

    unsigned long long dev = 0, ino = 0;
    if (sscanf(env, "%llu:%llu", &dev, &ino) != 2) {
        return false;
    }

    struct stat st;
    if (fstat(fd, &st) < 0) {
        return false;
    }
    return (dev_t)dev == st.st_dev && (ino_t)ino == st.st_ino;
}

// logfmt quoting: bare values are fine as long as they hold no whitespace,
// quote or '=', which would otherwise split or end the field.
bool needs_quoting(const char *s)
{
    if (*s == '\0') {
        return true;
    }
    for (const char *p = s; *p != '\0'; p++) {
        if (*p == ' ' || *p == '\t' || *p == '"' || *p == '=' || *p == '\\') {
            return true;
        }
    }
    return false;
}

void write_value(FILE *out, const char *s)
{
    if (!needs_quoting(s)) {
        fputs(s, out);
        return;
    }

    fputc('"', out);
    for (const char *p = s; *p != '\0'; p++) {
        if (*p == '"' || *p == '\\') {
            fputc('\\', out);
        }
        fputc(*p, out);
    }
    fputc('"', out);
}

void write_record(Level level, const char *msg, const char *fields)
{
    // Errors and warnings on stderr, the rest on stdout, as before.
    const bool to_stderr = (level <= Level::warn);
    FILE *out = to_stderr ? stderr : stdout;
    const bool journal = to_stderr ? g_journal_stderr : g_journal_stdout;
    const LevelInfo &li = info_for(level);

    fprintf(out, "%slevel=%s msg=", journal ? li.journal_prefix : "", li.name);
    write_value(out, msg);
    if (fields != NULL && *fields != '\0') {
        fputc(' ', out);
        fputs(fields, out);
    }
    fputc('\n', out);
}

}  // namespace

bool parse_level(const char *name, Level *out)
{
    for (size_t i = 0; i < sizeof(LEVELS) / sizeof(LEVELS[0]); i++) {
        if (strcmp(name, LEVELS[i].name) == 0) {
            *out = static_cast<Level>(i);
            return true;
        }
    }
    return false;
}

const char *level_name(Level level)
{
    return info_for(level).name;
}

void init(Level level)
{
    g_level = level;
    g_journal_stdout = is_journal_stream(fileno(stdout));
    g_journal_stderr = is_journal_stream(fileno(stderr));
}

bool enabled(Level level)
{
    return level <= g_level;
}

void write(Level level, const char *msg)
{
    write_record(level, msg, NULL);
}

void write_kv(Level level, const char *msg, const char *fields, ...)
{
    char rendered[512];
    va_list ap;

    va_start(ap, fields);
    vsnprintf(rendered, sizeof(rendered), fields, ap);
    va_end(ap);

    write_record(level, msg, rendered);
}

}  // namespace logging
