VERSION := 0.1.0
ARCH    := $(shell dpkg --print-architecture 2>/dev/null || echo arm64)
PREFIX  ?= /usr
CFLAGS  ?= -O2 -Wall -Wextra
PKG     := build/fandyn_$(VERSION)_$(ARCH).deb

all: fandynd

fandynd: src/fandynd.c
	$(CC) $(CFLAGS) -DVERSION=\"$(VERSION)\" -o $@ $< -lm

install: fandynd
	install -Dm755 fandynd        $(DESTDIR)$(PREFIX)/sbin/fandynd
	install -Dm644 fandyn.conf    $(DESTDIR)/etc/fandyn/fandyn.conf
	install -Dm644 fandyn.service $(DESTDIR)/lib/systemd/system/fandyn.service

deb: fandynd
	rm -rf build/pkg
	$(MAKE) install DESTDIR=build/pkg
	strip build/pkg$(PREFIX)/sbin/fandynd
	mkdir -p build/pkg/DEBIAN
	sed 's/@VERSION@/$(VERSION)/; s/@ARCH@/$(ARCH)/' debian/control > build/pkg/DEBIAN/control
	cp debian/conffiles debian/postinst debian/prerm debian/postrm build/pkg/DEBIAN/
	dpkg-deb --root-owner-group --build build/pkg $(PKG)
	@echo "built $(PKG)  ->  sudo apt install ./$(PKG)"

clean:
	rm -rf fandynd build

.PHONY: all install deb clean
