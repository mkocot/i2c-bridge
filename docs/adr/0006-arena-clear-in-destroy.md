# Arena clear-in-destroy is safe under single-allocator and capacity invariants

The sensor arena is one shared 128-byte bump allocator, and `sensor_factory_bmp280_destroy`
calls `arena_clear` on it. That reads like a use-after-clear, but it is safe while two
invariants hold: only BMP280 allocates from the arena (every other factory returns a
static instance, so the arena holds no other live pointer), and one BMP280 sensor
(`any_sensor_t` 16 bytes + `bmp280_handle_t` 72 bytes = 88 bytes) fits in the pool while
two do not (176 > 128), so at most one arena pointer is ever live and it is the one
being retired. A per-bank arena split was rejected on RAM grounds. The invariants are
recorded in GLOSSARY.md so that adding a second arena allocator or removing the clear is
not mistaken for a fix; doing either silently turns the arena into a use-after-clear.
