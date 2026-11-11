/*
 * Rocket design and RASP .eng motor parser. See rocket_design.h and
 * simulation/rockets/README.md.
 */

#include "rocket_design.h"

#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef enum {
    RULE_ANY,
    RULE_NON_NEGATIVE,
    RULE_POSITIVE,
} rule_t;

typedef struct {
    const char *name;
    size_t offset; /* of the first double in struct rocket_design */
    int count;
    rule_t rule;
    int optional;
} design_key_t;

#define FIELD(m) offsetof(struct rocket_design, m)

/* Numeric keys in the order of the key table (README.md). */
static const design_key_t keys[] = {
    {"burn_time_s", FIELD(burn_time_s), 1, RULE_POSITIVE, 1},
    {"mass_full_kg", FIELD(mass_full_kg), 1, RULE_POSITIVE, 0},
    {"mass_fuel_kg", FIELD(mass_fuel_kg), 1, RULE_POSITIVE, 0},
    {"inertia_full_kgm2", FIELD(inertia_full_kgm2), 3, RULE_ANY, 0},
    {"inertia_empty_kgm2", FIELD(inertia_empty_kgm2), 3, RULE_ANY, 0},
    {"inertia_full_products_kgm2",
     FIELD(inertia_full_products_kgm2),
     3,
     RULE_ANY,
     1},
    {"inertia_empty_products_kgm2",
     FIELD(inertia_empty_products_kgm2),
     3,
     RULE_ANY,
     1},
    {"length_m", FIELD(length_m), 1, RULE_POSITIVE, 0},
    {"frontal_area_m2", FIELD(frontal_area_m2), 1, RULE_POSITIVE, 0},
    {"cd0", FIELD(cd0), 1, RULE_NON_NEGATIVE, 0},
    {"cn_alpha", FIELD(cn_alpha), 1, RULE_NON_NEGATIVE, 0},
    {"cp_aft_of_cg_m", FIELD(cp_aft_of_cg_m), 1, RULE_ANY, 0},
    {"cmq", FIELD(cmq), 1, RULE_ANY, 0},
    {"cnr", FIELD(cnr), 1, RULE_ANY, 0},
    {"clp", FIELD(clp), 1, RULE_ANY, 0},
    {"roll_ref_length_m", FIELD(roll_ref_length_m), 1, RULE_POSITIVE, 0},
    {"canard_area_m2", FIELD(canard_area_m2), 1, RULE_POSITIVE, 0},
    {"canard_cl_alpha", FIELD(canard_cl_alpha), 1, RULE_NON_NEGATIVE, 0},
    {"canard_fwd_of_cg_m", FIELD(canard_fwd_of_cg_m), 1, RULE_ANY, 0},
    {"canard_lift_radius_m", FIELD(canard_lift_radius_m), 1, RULE_POSITIVE, 0},
    {"servo_tau_s", FIELD(servo_tau_s), 1, RULE_POSITIVE, 0},
    {"servo_rate_limit_dps", FIELD(servo_rate_limit_dps), 1, RULE_POSITIVE, 0},
};

#define KEY_COUNT (sizeof(keys) / sizeof(keys[0]))
#define MAX_VALUES 3

static int fail(char *err, size_t err_len, const char *fmt, ...) {
    va_list ap;

    va_start(ap, fmt);
    vsnprintf(err, err_len, fmt, ap);
    va_end(ap);
    return -1;
}

/* One line of a text buffer, without its line ending. */
typedef struct {
    const char *text;
    size_t len;
    size_t pos;
    int number;
} line_reader_t;

static int next_line(line_reader_t *r, char *line, size_t cap, int *truncated) {
    size_t n = 0;

    if (r->pos >= r->len) {
        return 0;
    }
    *truncated = 0;
    while (r->pos < r->len && r->text[r->pos] != '\n') {
        if (n + 1 < cap) {
            line[n++] = r->text[r->pos];
        } else {
            *truncated = 1;
        }
        r->pos++;
    }
    r->pos++; /* the '\n' */
    if (n > 0 && line[n - 1] == '\r') {
        n--;
    }
    line[n] = '\0';
    r->number++;
    return 1;
}

static char *trim(char *s) {
    char *end;

    while (*s == ' ' || *s == '\t') {
        s++;
    }
    end = s + strlen(s);
    while (end > s && (end[-1] == ' ' || end[-1] == '\t')) {
        *--end = '\0';
    }
    return s;
}

/* Whole token as a finite number. */
static int parse_number(const char *token, double *out) {
    char *end;
    double v;

    if (*token == '\0') {
        return -1;
    }
    v = strtod(token, &end);
    if (*end != '\0' || !isfinite(v)) {
        return -1;
    }
    *out = v;
    return 0;
}

static int in_rule(double v, rule_t rule) {
    switch (rule) {
        case RULE_NON_NEGATIVE:
            return v >= 0.0;
        case RULE_POSITIVE:
            return v > 0.0;
        default:
            return 1;
    }
}

static int valid_name(const char *s) {
    if (*s == '\0' || strlen(s) >= sizeof(((struct rocket_design *)0)->name)) {
        return 0;
    }
    for (; *s != '\0'; s++) {
        if (!((*s >= 'a' && *s <= 'z') || (*s >= '0' && *s <= '9')
              || *s == '_')) {
            return 0;
        }
    }
    return 1;
}

static int valid_motor_path(const char *s) {
    const char *p = s;

    if (*s == '\0' || *s == '/'
        || strlen(s) >= sizeof(((struct rocket_design *)0)->motor)) {
        return 0;
    }
    while (*p != '\0') {
        const char *slash = strchr(p, '/');
        size_t n = slash ? (size_t)(slash - p) : strlen(p);

        if (n == 2 && p[0] == '.' && p[1] == '.') {
            return 0;
        }
        p += n;
        if (*p == '/') {
            p++;
        }
    }
    return 1;
}

int rocket_design_parse(
    const char *text,
    size_t len,
    const char *path,
    struct rocket_design *out,
    char *err,
    size_t err_len
) {
    line_reader_t r = {text, len, 0, 0};
    char line[1024];
    int truncated;
    int seen[KEY_COUNT] = {0};
    int seen_name = 0;
    int seen_motor = 0;

    if (len > ROCKET_FILE_MAX) {
        return fail(err, err_len, "%s: file too large", path);
    }
    memset(out, 0, sizeof(*out));

    while (next_line(&r, line, sizeof(line), &truncated)) {
        char *hash = strchr(line, '#');
        char *eq;
        char *key;
        char *value;
        size_t k;

        if (hash != NULL) {
            *hash = '\0';
        }
        key = trim(line);
        if (*key == '\0') {
            continue;
        }
        eq = strchr(key, '=');
        if (eq == NULL || truncated) {
            return fail(
                err,
                err_len,
                "%s:%d: expected key = value",
                path,
                r.number
            );
        }
        *eq = '\0';
        value = trim(eq + 1);
        key = trim(key);

        if (strcmp(key, "name") == 0 || strcmp(key, "motor") == 0) {
            int is_name = key[0] == 'n';
            int *flag = is_name ? &seen_name : &seen_motor;

            if (*flag) {
                return fail(
                    err,
                    err_len,
                    "%s:%d: duplicate key %s",
                    path,
                    r.number,
                    key
                );
            }
            *flag = 1;
            if (is_name) {
                if (!valid_name(value)) {
                    return fail(
                        err,
                        err_len,
                        "%s:%d: bad name",
                        path,
                        r.number
                    );
                }
                strcpy(out->name, value);
            } else {
                if (!valid_motor_path(value)) {
                    return fail(
                        err,
                        err_len,
                        "%s:%d: motor path must be relative without ..",
                        path,
                        r.number
                    );
                }
                strcpy(out->motor, value);
            }
            continue;
        }

        for (k = 0; k < KEY_COUNT; k++) {
            if (strcmp(key, keys[k].name) == 0) {
                break;
            }
        }
        if (k == KEY_COUNT) {
            return fail(
                err,
                err_len,
                "%s:%d: unknown key %s",
                path,
                r.number,
                key
            );
        }
        if (seen[k]) {
            return fail(
                err,
                err_len,
                "%s:%d: duplicate key %s",
                path,
                r.number,
                key
            );
        }
        seen[k] = 1;

        double values[MAX_VALUES];
        int count = 0;
        char *item = value;

        for (;;) {
            char *comma = strchr(item, ',');

            if (comma != NULL) {
                *comma = '\0';
            }
            if (count == keys[k].count) {
                return fail(
                    err,
                    err_len,
                    "%s:%d: expected %d values for %s",
                    path,
                    r.number,
                    keys[k].count,
                    key
                );
            }
            if (parse_number(trim(item), &values[count]) != 0) {
                return fail(
                    err,
                    err_len,
                    "%s:%d: bad number for %s",
                    path,
                    r.number,
                    key
                );
            }
            count++;
            if (comma == NULL) {
                break;
            }
            item = comma + 1;
        }
        if (count != keys[k].count) {
            return fail(
                err,
                err_len,
                "%s:%d: expected %d values for %s",
                path,
                r.number,
                keys[k].count,
                key
            );
        }
        for (int i = 0; i < count; i++) {
            if (!in_rule(values[i], keys[k].rule)) {
                return fail(
                    err,
                    err_len,
                    "%s:%d: %s out of range",
                    path,
                    r.number,
                    key
                );
            }
        }
        memcpy((char *)out + keys[k].offset, values, count * sizeof(double));
        if (keys[k].offset == FIELD(burn_time_s)) {
            out->burn_time_given = 1;
        }
    }

    if (!seen_name) {
        return fail(err, err_len, "%s: missing key name", path);
    }
    if (!seen_motor) {
        return fail(err, err_len, "%s: missing key motor", path);
    }
    for (size_t k = 0; k < KEY_COUNT; k++) {
        if (!seen[k] && !keys[k].optional) {
            return fail(err, err_len, "%s: missing key %s", path, keys[k].name);
        }
    }
    return 0;
}

/* Split on spaces and tabs; returns the number of fields (at most max). */
static int split_fields(char *s, char **fields, int max) {
    int n = 0;

    for (;;) {
        while (*s == ' ' || *s == '\t') {
            s++;
        }
        if (*s == '\0') {
            return n;
        }
        if (n < max) {
            fields[n] = s;
        }
        n++;
        while (*s != '\0' && *s != ' ' && *s != '\t') {
            s++;
        }
        if (*s != '\0') {
            *s++ = '\0';
        }
    }
}

#define ENG_HEADER_FIELDS 7
#define ENG_MAX_FILE_POINTS (ROCKET_THRUST_POINTS - 1)

int rocket_motor_parse_eng(
    const char *text,
    size_t len,
    const char *path,
    struct rocket_design *out,
    char *err,
    size_t err_len
) {
    line_reader_t r = {text, len, 0, 0};
    char line[1024];
    int truncated;
    int have_header = 0;
    int count = 0;
    double t[ENG_MAX_FILE_POINTS];
    double n[ENG_MAX_FILE_POINTS];

    if (len > ROCKET_FILE_MAX) {
        return fail(err, err_len, "%s: file too large", path);
    }
    while (next_line(&r, line, sizeof(line), &truncated)) {
        char *semi = strchr(line, ';');
        char *fields[ENG_HEADER_FIELDS];
        int nf;

        if (semi != NULL) {
            *semi = '\0';
        }
        nf = split_fields(line, fields, ENG_HEADER_FIELDS);
        if (nf == 0) {
            continue;
        }
        if (!have_header) {
            if (nf != ENG_HEADER_FIELDS || truncated) {
                return fail(
                    err,
                    err_len,
                    "%s:%d: missing motor header",
                    path,
                    r.number
                );
            }
            have_header = 1;
            continue;
        }
        double tv;
        double nv;

        if (nf != 2 || truncated || parse_number(fields[0], &tv) != 0
            || parse_number(fields[1], &nv) != 0 || tv < 0.0) {
            if (nf == ENG_HEADER_FIELDS && count > 0) {
                return fail(
                    err,
                    err_len,
                    "%s:%d: file holds more than one motor",
                    path,
                    r.number
                );
            }
            return fail(
                err,
                err_len,
                "%s:%d: bad thrust point",
                path,
                r.number
            );
        }
        if (count == ENG_MAX_FILE_POINTS) {
            return fail(
                err,
                err_len,
                "%s:%d: too many thrust points (max %d)",
                path,
                r.number,
                ENG_MAX_FILE_POINTS
            );
        }
        if (count > 0 && tv <= t[count - 1]) {
            return fail(
                err,
                err_len,
                "%s:%d: thrust times must increase",
                path,
                r.number
            );
        }
        if (nv < 0.0) {
            return fail(
                err,
                err_len,
                "%s:%d: thrust must be >= 0",
                path,
                r.number
            );
        }
        t[count] = tv;
        n[count] = nv;
        count++;
    }
    if (!have_header) {
        return fail(err, err_len, "%s: missing motor header", path);
    }
    if (count < 2) {
        return fail(err, err_len, "%s: need at least 2 thrust points", path);
    }

    int k = 0;

    if (t[0] > 0.0) {
        out->thrust_t_s[k] = 0.0;
        out->thrust_n[k] = 0.0;
        k++;
    }
    for (int i = 0; i < count; i++, k++) {
        out->thrust_t_s[k] = t[i];
        out->thrust_n[k] = n[i];
    }
    out->thrust_count = k;
    for (int pad = 1; k < ROCKET_THRUST_POINTS; k++, pad++) {
        out->thrust_t_s[k] = t[count - 1] + pad;
        out->thrust_n[k] = 0.0;
    }
    return 0;
}

/* Leading principal minors of J = [a xy xz; xy b yz; xz yz c]. */
static int positive_definite(const double diag[3], const double prod[3]) {
    const double a = diag[0], b = diag[1], c = diag[2];
    const double xy = prod[0], xz = prod[1], yz = prod[2];
    const double det = a * (b * c - yz * yz) - xy * (xy * c - yz * xz)
        + xz * (xy * yz - b * xz);

    return a > 0.0 && a * b - xy * xy > 0.0 && det > 0.0;
}

int rocket_design_check(struct rocket_design *d, char *err, size_t err_len) {
    if (!d->burn_time_given) {
        d->burn_time_s = d->thrust_t_s[d->thrust_count - 1];
    }
    if (!(d->mass_fuel_kg < d->mass_full_kg)) {
        return fail(
            err,
            err_len,
            "mass_fuel_kg must be less than mass_full_kg"
        );
    }
    if (!positive_definite(
            d->inertia_full_kgm2,
            d->inertia_full_products_kgm2
        )) {
        return fail(err, err_len, "inertia_full_kgm2 is not positive-definite");
    }
    if (!positive_definite(
            d->inertia_empty_kgm2,
            d->inertia_empty_products_kgm2
        )) {
        return fail(
            err,
            err_len,
            "inertia_empty_kgm2 is not positive-definite"
        );
    }
    return 0;
}

typedef struct {
    char *buf;
    size_t len;
    size_t used;
    int overflow;
} out_t;

static void emit(out_t *o, const char *fmt, ...) {
    va_list ap;
    int n;

    if (o->overflow) {
        return;
    }
    va_start(ap, fmt);
    n = vsnprintf(o->buf + o->used, o->len - o->used, fmt, ap);
    va_end(ap);
    if (n < 0 || (size_t)n >= o->len - o->used) {
        o->overflow = 1;
        return;
    }
    o->used += (size_t)n;
}

int rocket_design_format(const struct rocket_design *d, char *buf, size_t len) {
    out_t o = {buf, len, 0, 0};

    if (len == 0) {
        return -1;
    }
    buf[0] = '\0';
    emit(&o, "name = %s\nmotor = %s\n", d->name, d->motor);
    for (size_t k = 0; k < KEY_COUNT; k++) {
        const double *v = (const double *)((const char *)d + keys[k].offset);

        emit(&o, "%s = ", keys[k].name);
        for (int i = 0; i < keys[k].count; i++) {
            emit(&o, i == 0 ? "%.17g" : ", %.17g", v[i]);
        }
        emit(&o, "\n");
    }
    for (int i = 0; i < d->thrust_count; i++) {
        emit(
            &o,
            "thrust_point = %.17g, %.17g\n",
            d->thrust_t_s[i],
            d->thrust_n[i]
        );
    }
    return o.overflow ? -1 : 0;
}

int rocket_design_motor_path(
    const char *rocket_path,
    const struct rocket_design *d,
    char *out,
    size_t len
) {
    const char *slash = strrchr(rocket_path, '/');
    size_t dir_len = slash ? (size_t)(slash - rocket_path) + 1 : 0;
    int n = snprintf(out, len, "%.*s%s", (int)dir_len, rocket_path, d->motor);

    return n < 0 || (size_t)n >= len ? -1 : 0;
}

size_t rocket_design_sizeof(void) {
    return sizeof(struct rocket_design);
}
