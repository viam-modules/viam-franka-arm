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

CGO_CFLAGS  := -I$(VENDOR_DIR)/include
CGO_LDFLAGS := -L$(VENDOR_DIR)/lib -lfranka -Wl,-rpath,\$$ORIGIN

export CGO_CFLAGS
export CGO_LDFLAGS

.PHONY: build module clean third_party-arm64 lint test gofmt tool-install

build:
	@test -d "$(VENDOR_DIR)" || (echo "Missing $(VENDOR_DIR). Run 'make third_party-arm64' (or your host arch equivalent) first." && exit 1)
	rm -rf $(BIN_OUTPUT_PATH)
	go build -o $(BIN_OUTPUT_PATH)/viam-franka-arm
	# Copy runtime shared libs next to the binary so $ORIGIN rpath finds them.
	mkdir -p $(BIN_OUTPUT_PATH)/lib
	cp -a $(VENDOR_DIR)/lib/*.so* $(BIN_OUTPUT_PATH)/lib/ 2>/dev/null || true
	# patchelf so the binary loads bundled libs from ./lib next to itself
	command -v patchelf >/dev/null && patchelf --set-rpath '$$ORIGIN/lib' $(BIN_OUTPUT_PATH)/viam-franka-arm || true

module: build
	rm -f $(BIN_OUTPUT_PATH)/module.tar.gz
	tar czf $(BIN_OUTPUT_PATH)/module.tar.gz \
		-C $(BIN_OUTPUT_PATH) viam-franka-arm lib \
		-C $(CURDIR) meta.json \
		$(if $(wildcard arm/*.urdf),arm/$(notdir $(wildcard arm/*.urdf)),)

# Build vendored libfranka 0.9.2 + deps for linux/arm64 via Docker buildx.
third_party-arm64:
	bash third_party/build.sh linux/arm64

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
