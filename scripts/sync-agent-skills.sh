#!/usr/bin/env bash
# Sync the upstream-managed engineering skills in .agents/skills/ from mattpocock/skills.
#
# Managed skills are copied from upstream and their skills-lock.json entries are recomputed.
# Repo-local skills (robot-wifi-setup) are never touched.
#
# Usage:
#   scripts/sync-agent-skills.sh [upstream-checkout-dir]
#
# With no argument it clones mattpocock/skills (shallow) into a temp dir itself.
# Exits 0 with "changed" or "unchanged" on stdout so a workflow can branch on it.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$REPO_ROOT"

# Upstream path -> local skill dir name. Only these are managed; everything else in
# .agents/skills/ is repo-local and left alone.
MANAGED="
skills/engineering/code-review:code-review
skills/engineering/codebase-design:codebase-design
skills/engineering/diagnosing-bugs:diagnosing-bugs
skills/engineering/grill-with-docs:grill-with-docs
skills/engineering/implement:implement
skills/engineering/improve-codebase-architecture:improve-codebase-architecture
skills/engineering/pr:pr
skills/engineering/tdd:tdd
skills/engineering/triage:triage
skills/misc/git-guardrails-claude-code:git-guardrails-claude-code
skills/misc/setup-pre-commit:setup-pre-commit
skills/productivity/grill-me:grill-me
skills/productivity/grilling:grilling
skills/productivity/handoff:handoff
skills/productivity/teach:teach
skills/productivity/writing-for-agents:writing-for-agents
"

UPSTREAM="${1:-}"
TMP=""
cleanup() { [ -n "$TMP" ] && rm -rf "$TMP"; }
trap cleanup EXIT

if [ -z "$UPSTREAM" ]; then
  TMP="$(mktemp -d)"
  git clone --depth 1 --quiet https://github.com/mattpocock/skills "$TMP/skills" >/dev/null
  UPSTREAM="$TMP/skills"
fi

[ -d "$UPSTREAM/skills" ] || { echo "error: $UPSTREAM does not look like mattpocock/skills" >&2; exit 2; }

for entry in $MANAGED; do
  src_rel="${entry%%:*}"
  name="${entry##*:}"
  src="$UPSTREAM/$src_rel"
  if [ ! -f "$src/SKILL.md" ]; then
    echo "warning: upstream is missing $src_rel — leaving .agents/skills/$name alone" >&2
    continue
  fi
  rm -rf ".agents/skills/$name"
  mkdir -p ".agents/skills/$name"
  # Copy the skill itself (SKILL.md plus any supporting files it ships).
  (cd "$src" && tar -cf - .) | (cd ".agents/skills/$name" && tar -xf -)
done

python3 - "$UPSTREAM" <<'PY'
import hashlib
import json
import pathlib
import sys

upstream = pathlib.Path(sys.argv[1])
managed = {
    "code-review": "skills/engineering/code-review/SKILL.md",
    "codebase-design": "skills/engineering/codebase-design/SKILL.md",
    "diagnosing-bugs": "skills/engineering/diagnosing-bugs/SKILL.md",
    "grill-with-docs": "skills/engineering/grill-with-docs/SKILL.md",
    "implement": "skills/engineering/implement/SKILL.md",
    "improve-codebase-architecture": "skills/engineering/improve-codebase-architecture/SKILL.md",
    "pr": "skills/engineering/pr/SKILL.md",
    "tdd": "skills/engineering/tdd/SKILL.md",
    "triage": "skills/engineering/triage/SKILL.md",
    "git-guardrails-claude-code": "skills/misc/git-guardrails-claude-code/SKILL.md",
    "setup-pre-commit": "skills/misc/setup-pre-commit/SKILL.md",
    "grill-me": "skills/productivity/grill-me/SKILL.md",
    "grilling": "skills/productivity/grilling/SKILL.md",
    "handoff": "skills/productivity/handoff/SKILL.md",
    "teach": "skills/productivity/teach/SKILL.md",
    "writing-for-agents": "skills/productivity/writing-for-agents/SKILL.md",
}

lock_path = pathlib.Path("skills-lock.json")
lock = json.loads(lock_path.read_text())
for name, skill_path in managed.items():
    local = pathlib.Path(".agents/skills", name, "SKILL.md")
    if not local.exists():
        continue
    # Hash the upstream file, matching the content we just installed.
    digest = hashlib.sha256(local.read_bytes()).hexdigest()
    lock["skills"][name] = {
        "source": "mattpocock/skills",
        "sourceType": "github",
        "skillPath": skill_path,
        "computedHash": digest,
    }
lock["skills"] = {k: lock["skills"][k] for k in sorted(lock["skills"])}
lock_path.write_text(json.dumps(lock, indent=2) + "\n")
print(f"lock: {len(lock['skills'])} skills")
PY

if [ -n "$(git status --porcelain .agents/skills skills-lock.json)" ]; then
  echo "changed"
else
  echo "unchanged"
fi
