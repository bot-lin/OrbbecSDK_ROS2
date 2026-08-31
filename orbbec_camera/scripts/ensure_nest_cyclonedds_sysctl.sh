#!/usr/bin/env bash

set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  ensure_nest_cyclonedds_sysctl.sh [options] <controller-host>

Ensure that a NEST controller has the host-side CycloneDDS receive-buffer
configuration and apply it. Existing conflicting files are never overwritten.

Options:
  --check                 Check only; do not create or apply anything
  --port <port>           SSH port (default: 6364)
  --user <user>           SSH user (default: root)
  --identity-file <path>  SSH private key (default: $NEST_SSH_KEY or ~/.ssh/id_ed25519)
  -h, --help              Show this help

Examples:
  ensure_nest_cyclonedds_sysctl.sh 10.148.165.8
  ensure_nest_cyclonedds_sysctl.sh --check 10.148.165.8
EOF
}

fail() {
  printf 'Error: %s\n' "$*" >&2
  exit 1
}

operation="ensure"
ssh_port="6364"
ssh_user="root"
identity_file="${NEST_SSH_KEY:-${HOME}/.ssh/id_ed25519}"
controller_host=""

while (($# > 0)); do
  case "$1" in
    --check)
      operation="check"
      ;;
    --port)
      (($# >= 2)) || fail "--port requires a value"
      ssh_port="$2"
      shift
      ;;
    --user)
      (($# >= 2)) || fail "--user requires a value"
      ssh_user="$2"
      shift
      ;;
    --identity-file)
      (($# >= 2)) || fail "--identity-file requires a path"
      identity_file="$2"
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    --*)
      fail "unknown option: $1"
      ;;
    *)
      [[ -z "$controller_host" ]] || fail "only one controller host may be supplied"
      controller_host="$1"
      ;;
  esac
  shift
done

[[ -n "$controller_host" ]] || {
  usage >&2
  exit 1
}
[[ "$controller_host" =~ ^[A-Za-z0-9._-]+$ ]] || fail "invalid controller host: $controller_host"
[[ "$ssh_user" =~ ^[A-Za-z_][A-Za-z0-9_-]*$ ]] || fail "invalid SSH user: $ssh_user"
[[ "$ssh_port" =~ ^[0-9]+$ ]] || fail "SSH port must be numeric"
((ssh_port >= 1 && ssh_port <= 65535)) || fail "SSH port must be between 1 and 65535"
[[ -r "$identity_file" ]] || fail "SSH identity file is not readable: $identity_file"

readonly config_path="/etc/sysctl.d/99-nest-cyclonedds.conf"
readonly desired_rmem_max="8388608"
readonly ssh_target="${ssh_user}@${controller_host}"

printf 'Controller: %s:%s\n' "$ssh_target" "$ssh_port"

ssh \
  -p "$ssh_port" \
  -i "$identity_file" \
  -o BatchMode=yes \
  -o ConnectTimeout=10 \
  -o StrictHostKeyChecking=no \
  "$ssh_target" \
  bash -s -- "$operation" "$config_path" "$desired_rmem_max" <<'REMOTE_SCRIPT'
set -euo pipefail

operation="$1"
config_path="$2"
desired_rmem_max="$3"

if [[ "$(id -u)" -ne 0 ]]; then
  printf 'Error: remote user must be root to manage %s\n' "$config_path" >&2
  exit 1
fi

printf 'Remote host: %s\n' "$(hostname)"

validate_existing_file() {
  if [[ -L "$config_path" || ! -f "$config_path" ]]; then
    printf 'Error: refusing to use non-regular config path: %s\n' "$config_path" >&2
    exit 3
  fi

  local assignment_pattern
  local assignment_count
  local unexpected_lines
  assignment_pattern="^[[:space:]]*net\\.core\\.rmem_max[[:space:]]*=[[:space:]]*${desired_rmem_max}[[:space:]]*$"
  assignment_count="$(grep -Ec "$assignment_pattern" "$config_path" || true)"
  unexpected_lines="$(grep -Ev "^[[:space:]]*(#|$)|${assignment_pattern}" "$config_path" || true)"
  if [[ "$assignment_count" -ne 1 || -n "$unexpected_lines" ]]; then
    printf 'Error: existing config differs; refusing to overwrite %s\n' "$config_path" >&2
    sed -n '1,80p' "$config_path" >&2
    exit 3
  fi
}

if [[ -e "$config_path" || -L "$config_path" ]]; then
  validate_existing_file
  printf 'Config: present and valid (%s)\n' "$config_path"
else
  if [[ "$operation" == "check" ]]; then
    printf 'Config: missing (%s)\n' "$config_path" >&2
    exit 2
  fi

  staged_path="$(mktemp /tmp/99-nest-cyclonedds.conf.XXXXXX)"
  trap 'rm -f "$staged_path"' EXIT
  printf '%s\n' \
    '# Allow CycloneDDS to obtain its requested receive buffer for large ROS 2 samples.' \
    '# A 640x360 RGB8 frame is 691200 bytes; the previous 208 KiB ceiling caused UDP drops.' \
    "net.core.rmem_max = ${desired_rmem_max}" \
    >"$staged_path"
  install -o root -g root -m 0644 "$staged_path" "$config_path"
  printf 'Config: created (%s)\n' "$config_path"
fi

if [[ "$operation" == "check" ]]; then
  live_rmem_max="$(sysctl -n net.core.rmem_max)"
  if [[ "$live_rmem_max" != "$desired_rmem_max" ]]; then
    printf 'Live value: %s (expected %s); run without --check to apply the config\n' \
      "$live_rmem_max" "$desired_rmem_max" >&2
    exit 4
  fi
else
  sysctl -p "$config_path"
  live_rmem_max="$(sysctl -n net.core.rmem_max)"
  [[ "$live_rmem_max" == "$desired_rmem_max" ]] || {
    printf 'Error: live net.core.rmem_max is %s, expected %s\n' \
      "$live_rmem_max" "$desired_rmem_max" >&2
    exit 5
  }
fi

printf 'Live net.core.rmem_max: %s\n' "$live_rmem_max"
printf 'Result: %s\n' "$operation"
REMOTE_SCRIPT
