BIN_OUTPUT_PATH := bin
TOOL_BIN := bin/gotools/$(shell uname -s)-$(shell uname -m)
PATH_WITH_TOOLS := "`pwd`/$(TOOL_BIN):${PATH}"

UNAME_S ?= $(shell uname -s)
UNAME_M ?= $(shell uname -m)

# Map host to vendored third_party dir.
ifeq ($(UNAME_S),Linux)
	ifeq ($(UNAME_M),aarch64)
		VENDOR_TRIPLE := linux-arm64
	else ifeq ($(UNAME_M),x86_64)
		VENDOR_TRIPLE := linux-amd64
	endif
endif
ifeq ($(UNAME_S),Darwin)
	ifeq ($(UNAME_M),arm64)
		VENDOR_TRIPLE := darwin-arm64
	endif
endif

VENDOR_DIR := $(CURDIR)/third_party/$(VENDOR_TRIPLE)

CGO_CFLAGS   := -I$(VENDOR_DIR)/include
CGO_CXXFLAGS := -I$(VENDOR_DIR)/include -std=c++17
CGO_LDFLAGS  := -L$(VENDOR_DIR)/lib -lfranka -Wl,-rpath-link,$(VENDOR_DIR)/lib -Wl,-rpath,\$$ORIGIN/lib

export CGO_CFLAGS
export CGO_CXXFLAGS
export CGO_LDFLAGS

.PHONY: build module module.tar.gz setup clean third_party-arm64 third_party-amd64 lint test gofmt tool-install

build:
	@test -d "$(VENDOR_DIR)" || (echo "Missing $(VENDOR_DIR)." && echo "Run one of:" && echo "  make third_party-$(subst linux-,,$(VENDOR_TRIPLE))   # buildx (needs docker/podman)" && echo "  make setup                       # native build (needs sudo for apt)" && exit 1)
	rm -rf $(BIN_OUTPUT_PATH)
	go build -o $(BIN_OUTPUT_PATH)/viam-franka-arm
	# Copy runtime shared libs next to the binary so $ORIGIN rpath finds them.
	mkdir -p $(BIN_OUTPUT_PATH)/lib
	cp -a $(VENDOR_DIR)/lib/*.so* $(BIN_OUTPUT_PATH)/lib/ 2>/dev/null || true
	# Rewrite any absolute symlinks (e.g. libpcre.so -> /lib/x86_64-linux-gnu/libpcre.so.3)
	# to relative ones so the bundle is self-contained. `viam module upload` warns
	# about absolute symlinks because they break on deploy hosts lacking that path.
	@for link in $(BIN_OUTPUT_PATH)/lib/*.so*; do \
		[ -L "$$link" ] || continue; \
		target=$$(readlink "$$link"); \
		case "$$target" in \
			/*) base=$$(basename "$$target"); \
			    if [ -e "$(BIN_OUTPUT_PATH)/lib/$$base" ]; then \
			        ln -sfn "$$base" "$$link"; \
			    else \
			        rm "$$link"; \
			    fi ;; \
		esac; \
	done
	# patchelf the binary's RPATH so its bundled libs find their *own* transitive
	# deps inside ./lib too. --force-rpath emits DT_RPATH (legacy, searched for
	# transitive deps) instead of DT_RUNPATH (modern, direct-deps-only).
	command -v patchelf >/dev/null && patchelf --force-rpath --set-rpath '$$ORIGIN/lib' $(BIN_OUTPUT_PATH)/viam-franka-arm || true
	# Also patchelf each bundled .so so its own transitive deps resolve from
	# the same dir, regardless of the binary's RPATH.
	command -v patchelf >/dev/null && for so in $(BIN_OUTPUT_PATH)/lib/*.so*; do test -L "$$so" || patchelf --force-rpath --set-rpath '$$ORIGIN' "$$so" 2>/dev/null || true; done

module: build
	rm -f $(BIN_OUTPUT_PATH)/module.tar.gz
	tar czf $(BIN_OUTPUT_PATH)/module.tar.gz \
		$(BIN_OUTPUT_PATH)/viam-franka-arm \
		$(BIN_OUTPUT_PATH)/lib \
		meta.json \
		$(wildcard arm/*.urdf)

# Alias so `make module.tar.gz` works for users who type the artifact name.
module.tar.gz: module

# Native libfranka build (used by Viam cloud-build via meta.json:build.setup).
# Locally, runs only on the host's own arch.
setup:
	./setup.sh

# Cross-arch builds via Docker/podman buildx for local dev convenience.
third_party-arm64:
	bash third_party/build.sh linux/arm64

third_party-amd64:
	bash third_party/build.sh linux/amd64

clean:
	rm -rf $(BIN_OUTPUT_PATH)

tool-install:
	GOBIN=`pwd`/$(TOOL_BIN) go install \
		github.com/edaniels/golinters/cmd/combined \
		github.com/rhysd/actionlint/cmd/actionlint
	GOBIN=`pwd`/$(TOOL_BIN) GOTOOLCHAIN=go1.25.1 go install \
		github.com/golangci/golangci-lint/v2/cmd/golangci-lint@v2.8.0

gofmt:
	gofmt -w -s .

lint: gofmt tool-install
	go mod tidy
	PATH=$(PATH_WITH_TOOLS) golangci-lint run -c etc/.golangci.yaml --fix

test: tool-install
	go test -v -race -failfast ./...

upload:
	@echo viam module upload --version \"0.0.5\" --platform \"linux/amd64\" bin/module.tar.gz