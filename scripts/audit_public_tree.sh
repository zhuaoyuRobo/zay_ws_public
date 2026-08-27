#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${ROOT_DIR}"

status=0

if rg -n --hidden \
  --glob '!scripts/audit_public_tree.sh' \
  --glob '!.git/**' \
  '(BEGIN (RSA|OPENSSH|EC) PRIVATE KEY|password[[:space:]]*[:=]|api[_-]?key[[:space:]]*[:=]|/home/[^/[:space:]]+|[A-Za-z]:\\Users\\[^\\[:space:]]+)' .; then
  echo "Potential credential or workstation path found." >&2
  status=1
fi

if find . -type f \( \
  -name '*.pt' -o -name '*.pth' -o -name '*.onnx' -o -name '*.engine' -o \
  -name '*.bag' -o -name '*.db3' -o -name '*.mcap' -o -name '*.pcd' -o \
  -name '*.posegraph' -o -name '*.pyc' \) -print | grep -q .; then
  echo "Excluded binary, model, or experiment artifact found." >&2
  status=1
fi

exit "${status}"
