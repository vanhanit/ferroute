# ferroute developer workflow. Host needs only `make` and `docker`.
#   make help          list targets
#   make NATIVE=1 ...  run directly on the host / inside a CI container (no nested docker)
SHELL := bash
.SHELLFLAGS := -eu -o pipefail -c
.DEFAULT_GOAL := help

IMAGE        ?= ferroute-dev:local
DOCKER       ?= docker
NATIVE       ?= 0
KICAD_IMAGE  ?= kicad/kicad:10.0.8@sha256:3f4a01786dfbe803a65f345c3342b765db9add913f5e9448f8356793714ddc34
NODE_IMAGE   ?= node:24.22.3-trixie-slim@sha256:af02d57b57719b9657a45199a79264a9437940cd55b3c7b3fc99615654f124a0
RUNTIME_TAG  ?= ferroute:local
BOARDS       := $(basename $(notdir $(wildcard testdata/boards/*.kicad_pcb)))
UID_GID      := $(shell id -u):$(shell id -g)
PWD_         := $(CURDIR)

# RUN(cmd): execute cmd in the dev image (repo at /work, registry+target in named volumes,
# host uid/gid, HOME=/tmp so cargo/rustup caches stay writable). With NATIVE=1 run in place.
# Commands passed to RUN must not contain single quotes.
ifeq ($(NATIVE),1)
define RUN
bash -c '$(1)'
endef
else
define RUN
$(DOCKER) run --rm -u $(UID_GID) -e HOME=/tmp -e CARGO_TARGET_DIR=/cargo-target \
  -e WGPU_BACKEND -e RUST_BACKTRACE \
  -v "$(PWD_)":/work -w /work \
  -v ferroute-cargo-registry:/usr/local/cargo/registry \
  -v ferroute-cargo-target:/cargo-target \
  $(IMAGE) bash -c '$(1)'
endef
endif

# Cargo target dir as seen by commands executed through RUN (shell-expanded there).
TGT := $${CARGO_TARGET_DIR:-target}

.PHONY: help dev-image shell build test test-gpu fmt fmt-check lint bench drc-bench audit coverage dist \
        sonar sonar-up sonar-down e2e kicad10-drc docker-build release changelog hooks clean

help: ## Show this help
	@grep -E '^[a-zA-Z0-9_-]+:.*## ' $(MAKEFILE_LIST) | awk 'BEGIN{FS=":.*## "}{printf "  \033[36m%-14s\033[0m %s\n",$$1,$$2}'

dev-image: ## Build the dev toolchain image
	DOCKER_BUILDKIT=1 $(DOCKER) build --target dev -t $(IMAGE) .

shell: ## Interactive shell in the dev image
	$(DOCKER) run --rm -it -u $(UID_GID) -e HOME=/tmp -e CARGO_TARGET_DIR=/cargo-target \
	  -v "$(PWD_)":/work -w /work -v ferroute-cargo-registry:/usr/local/cargo/registry \
	  -v ferroute-cargo-target:/cargo-target $(IMAGE) bash

build: ## Build the workspace (debug)
	$(call RUN,cargo build --workspace --locked)

test: ## Unit/integration tests (nextest, falls back to cargo test) + doctests
	$(call RUN,if cargo nextest --version >/dev/null 2>&1; then cargo nextest run --workspace --locked; else cargo test --workspace --locked; fi; cargo test --doc --workspace --locked)

test-gpu: ## GPU-path tests on the lavapipe/Vulkan adapter
	$(call RUN,WGPU_BACKEND=vulkan cargo test -p ferroute-gpu --features ferroute-gpu/gpu --locked)

fmt: ## Format sources with rustfmt
	$(call RUN,cargo fmt --all)

fmt-check: ## Check formatting
	$(call RUN,cargo fmt --all -- --check)

lint: fmt-check ## rustfmt check + clippy (-D warnings) + toolchain alignment
	scripts/check_toolchain.sh
	$(call RUN,cargo clippy --workspace --all-targets --locked -- -D warnings)

bench: ## Criterion benchmarks
	$(call RUN,cargo bench -p ferroute-engine)

drc-bench: ## Routing/DRC regression suite against bench/baseline.json
	$(call RUN,mkdir -p out/bench && cargo run --release --locked -p ferroute-cli -- bench --suite ci --baseline bench/baseline.json --out out/bench)

audit: ## Licenses/bans/sources/advisories (cargo-deny) + RustSec (cargo-audit)
	$(call RUN,cargo deny check && cargo audit)

coverage: ## LCOV coverage to out/lcov.info
	$(call RUN,mkdir -p out && cargo llvm-cov --workspace --locked --lcov --output-path out/lcov.info)

sonar: coverage ## Coverage + clippy JSON, then sonar-scanner against local SonarQube
	$(call RUN,mkdir -p out && cargo clippy --workspace --all-targets --locked --message-format=json > out/clippy.json)
	$(DOCKER) compose --profile sonar run --rm sonar-scanner

sonar-up: ## Start local SonarQube (http://localhost:9000)
	$(DOCKER) compose --profile sonar up -d --wait sonarqube

sonar-down: ## Stop local SonarQube
	$(DOCKER) compose --profile sonar down

e2e: ## Route every testdata board (with testdata/rules/<board>.toml if present), then run ferroute's DRC
	$(call RUN,cargo build --release --locked -p ferroute-cli && mkdir -p out && for f in testdata/boards/*.kicad_pcb; do n=$$(basename $$f .kicad_pcb); echo "== $$n"; r=""; [ -f testdata/rules/$$n.toml ] && r="--rules testdata/rules/$$n.toml --strict-rules"; $(TGT)/release/ferroute route $$f -o out/$$n.routed.kicad_pcb --report out/$$n.json --svg out/$$n.svg --seed 1 $$r || exit 1; $(TGT)/release/ferroute drc out/$$n.routed.kicad_pcb --fail-on-violation; done)

kicad10-drc: ## Independent KiCad 10 DRC on routed boards (run `make e2e` first)
	@test -n "$(BOARDS)" || { echo "no boards in testdata/boards"; exit 1; }
	$(DOCKER) run --rm -u $(UID_GID) -e HOME=/tmp -v "$(PWD_)":/work -w /work $(KICAD_IMAGE) \
	  bash -c 'for n in $(BOARDS); do echo "== $$n"; kicad-cli pcb drc --format json --severity-error --exit-code-violations -o out/$$n.drc.json out/$$n.routed.kicad_pcb || true; done'
	$(call RUN,for n in $(BOARDS); do python3 -I scripts/check_kicad_drc.py out/$$n.drc.json; done)

docker-build: ## Build the runtime image
	DOCKER_BUILDKIT=1 $(DOCKER) build --target runtime -t $(RUNTIME_TAG) .

release: ## Static musl release binary into dist/
	$(call RUN,cargo build --release --locked --target x86_64-unknown-linux-musl -p ferroute-cli && mkdir -p dist && cp $(TGT)/x86_64-unknown-linux-musl/release/ferroute dist/ferroute-x86_64-unknown-linux-musl)

CROSS_IMAGE ?= ferroute-cross:local
TAG ?= dev
dist: ## Cross-compile + package every release target in the Linux `cross` container (TAG=vX.Y.Z)
ifeq ($(NATIVE),1)
	scripts/dist.sh $(TAG)
else
	DOCKER_BUILDKIT=1 $(DOCKER) build --target cross -t $(CROSS_IMAGE) .
	$(DOCKER) run --rm -u $(UID_GID) -e HOME=/tmp -e CARGO_HOME=/usr/local/cargo -v "$(PWD_)":/work -w /work \
	  -v ferroute-cargo-registry:/usr/local/cargo/registry $(CROSS_IMAGE) scripts/dist.sh $(TAG)
endif

changelog: ## Regenerate CHANGELOG.md with git-cliff
	$(call RUN,git cliff --config cliff.toml --output CHANGELOG.md)

hooks: ## Install the local commit-msg hook (conventional commits)
	install -m 0755 .github/hooks/commit-msg "$$(git rev-parse --git-path hooks)/commit-msg"
	@echo "installed commit-msg hook"

clean: ## Remove out/ and dist/ and the target volume
	$(RM) -r ./out ./dist
	-$(DOCKER) volume rm ferroute-cargo-target
