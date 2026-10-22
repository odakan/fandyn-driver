VERSION  := 1.0.0
ARCH     := $(shell dpkg --print-architecture 2>/dev/null || echo arm64)
PREFIX   ?= /usr
WARN     := -Wall -Wextra -Werror -Wshadow -Wformat=2 -Wconversion -Wundef -Wstrict-prototypes \
            -Wmissing-prototypes -Wcast-align -Wwrite-strings -Wvla
HARDEN   := -fstack-protector-strong -D_FORTIFY_SOURCE=2 -fPIE
CFLAGS   ?= -O2
CFLAGS   += -std=gnu99 $(WARN) $(HARDEN) -DFD_VERSION=\"$(VERSION)\"
LDFLAGS  += -pie -Wl,-z,relro,-z,now
PKG      := build/fandyn_$(VERSION)_$(ARCH).deb
BINS     := fandynd fandyn-calibrate

all: $(BINS)

fandynd: src/fandynd.c src/fandyn.h
	$(CC) $(CFLAGS) -o $@ $< $(LDFLAGS)

fandyn-calibrate: src/fandyn-calibrate.c src/fandyn.h
	$(CC) $(CFLAGS) -Wno-conversion -o $@ $< $(LDFLAGS) -lm -pthread

install: all
	install -Dm755 fandynd          "$(DESTDIR)$(PREFIX)/sbin/fandynd"
	install -Dm755 fandyn-calibrate "$(DESTDIR)$(PREFIX)/sbin/fandyn-calibrate"
	install -Dm644 systemd/fandyn.service           "$(DESTDIR)/lib/systemd/system/fandyn.service"
	install -Dm644 systemd/fandyn-calibrate.service "$(DESTDIR)/lib/systemd/system/fandyn-calibrate.service"
	install -Dm644 systemd/fandyn-calibrate.timer   "$(DESTDIR)/lib/systemd/system/fandyn-calibrate.timer"
	install -Dm644 README.md         "$(DESTDIR)$(PREFIX)/share/doc/fandyn/README.md"
	install -Dm644 debian/copyright  "$(DESTDIR)$(PREFIX)/share/doc/fandyn/copyright"
	install -d "$(DESTDIR)$(PREFIX)/share/doc/fandyn" "$(DESTDIR)$(PREFIX)/share/man/man8"
	gzip -9n < debian/changelog > "$(DESTDIR)$(PREFIX)/share/doc/fandyn/changelog.gz"
	gzip -9n < man/fandynd.8 > "$(DESTDIR)$(PREFIX)/share/man/man8/fandynd.8.gz"
	gzip -9n < man/fandyn-calibrate.8 > "$(DESTDIR)$(PREFIX)/share/man/man8/fandyn-calibrate.8.gz"
	chmod 644 "$(DESTDIR)$(PREFIX)/share/doc/fandyn/changelog.gz" "$(DESTDIR)$(PREFIX)/share/man/man8/"*.gz

# release package: refuses to build unless the versions agree and every test passes
deb: check-version test
	rm -rf build/pkg
	$(MAKE) install DESTDIR="$(CURDIR)/build/pkg"
	strip --strip-unneeded build/pkg$(PREFIX)/sbin/fandynd build/pkg$(PREFIX)/sbin/fandyn-calibrate
	mkdir -p build/pkg/DEBIAN
	sed -e 's/@VERSION@/$(VERSION)/' -e 's/@ARCH@/$(ARCH)/' \
	    -e "s/@SIZE@/$$(du -sk build/pkg | cut -f1)/" debian/control > build/pkg/DEBIAN/control
	install -m755 debian/postinst debian/prerm debian/postrm build/pkg/DEBIAN/
	cd build/pkg && find . -type f ! -path './DEBIAN/*' | sort | xargs md5sum | sed 's| \./| |' > DEBIAN/md5sums
	fakeroot dpkg-deb --build build/pkg $(PKG)
	@echo "built $(PKG)"
	@echo "install: sudo apt install ./$(PKG)"

check-version:
	@v=$$(sed -n '1s/^fandyn (\([^)]*\)).*/\1/p' debian/changelog); \
	if [ "$$v" != "$(VERSION)" ]; then echo "version mismatch: Makefile $(VERSION), debian/changelog $$v"; exit 1; fi

.PHONY: all clean install deb check-version
clean:
	rm -rf $(BINS) build

# ---- tests (not shipped)
build/test_core: tests/test_core.c src/fandyn.h
	@mkdir -p build
	$(CC) $(CFLAGS) -o $@ $< $(LDFLAGS)

build/alloc_count.so: tests/alloc_count.c
	@mkdir -p build
	$(CC) -O2 -fPIC -shared -o $@ $<

test: all build/test_core build/alloc_count.so
	./build/test_core
	./tests/test_runtime.sh

.PHONY: test
