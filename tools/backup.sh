#!/usr/bin/env bash
# Backup after each session (codebase plan §8; workbench stage 9.2).
#
#   BACKUP_DISK=/mnt/backup [BACKUP_OFFSITE=user@host:/path] bash tools/backup.sh [--dry-run]
#        [--data-dir DIR] [--result-json PATH]
#
# - .bin files of every dataset            -> the second disk and, if set, offsite;
# - .img files of eval and killtest trials -> the second disk and, if set, offsite;
# - the run folders (data/runs; their trial.bin hard links stay links, rsync -H) and the small
#   irreplaceable records (data/workbench, calib, params, latency, manifest.csv,
#   annotations.csv)                        -> both.
# Nothing at the destination is deleted or lost: a file that changed since the last backup is copied
# again, and the copy it replaces is moved to slosh-data/replaced/<time>/ (rsync --backup; pre-hardware
# review D4), so a damaged record never takes the place of a good copy. A changed .bin is reported
# loudly: trial files are never renamed or edited (A4), so it means damage.
# The destinations are [MISSING] (docs/open_questions.md H9): the script refuses until BACKUP_DISK is set.
set -euo pipefail

here="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
data="${SLOSH_DATA:-$here/data}"
dry=""
result=""
while [ $# -gt 0 ]; do
  case "$1" in
    --dry-run) dry="--dry-run"; shift ;;
    --data-dir) data="$2"; shift 2 ;;
    --result-json) result="$2"; shift 2 ;;
    *) echo "unknown argument: $1" >&2; exit 2 ;;
  esac
done

replaced_files=0
replaced_bin=0
replaced_dir=""
write_result() {   # ok bin img message
  if [ -n "$result" ]; then
    printf '{"backup_ok": %s, "bin_files": %s, "img_files": %s, "disk": "%s", "offsite": "%s", "dry_run": %s, "replaced_files": %s, "replaced_bin_files": %s, "replaced_dir": "%s", "message": "%s"}\n' \
      "$1" "$2" "$3" "${BACKUP_DISK:-}" "${BACKUP_OFFSITE:-}" "$([ -n "$dry" ] && echo true || echo false)" \
      "$replaced_files" "$replaced_bin" "$replaced_dir" "$4" > "$result"
  fi
}

if [ -z "${BACKUP_DISK:-}" ]; then
  echo "BACKUP_DISK is not set: the backup disk is [MISSING] (docs/open_questions.md H9)" >&2
  write_result false 0 0 "BACKUP_DISK not set"; exit 3
fi
if [ ! -d "$BACKUP_DISK" ]; then
  echo "BACKUP_DISK=$BACKUP_DISK is not a mounted directory" >&2
  write_result false 0 0 "backup disk not mounted"; exit 3
fi
command -v rsync >/dev/null || { echo "rsync is not installed" >&2; write_result false 0 0 "no rsync"; exit 3; }
[ -d "$data" ] || { echo "no data directory at $data" >&2; write_result false 0 0 "no data"; exit 3; }

list="$(mktemp)"; trap 'rm -f "$list"' EXIT
# .bin of every dataset; .img only where the trial's header says eval or killtest
python3 - "$data" > "$list" <<'EOF'
import json, pathlib, struct, sys
data = pathlib.Path(sys.argv[1])
for b in sorted(data.rglob("*.bin")):
    print(b.relative_to(data))
    img = b.with_suffix(".img")
    if img.exists():
        try:
            with open(b, "rb") as f:
                head = f.read(12)
                n = struct.unpack_from("<I", head, 8)[0]
                dataset = json.loads(f.read(n).decode("utf-8")).get("dataset")
        except Exception:
            dataset = None
        if dataset in ("eval", "killtest"):
            print(img.relative_to(data))
for small in ("runs", "workbench", "calib", "params", "latency", "manifest.csv", "annotations.csv"):
    p = data / small
    if p.is_dir():
        for f in sorted(p.rglob("*")):
            if f.is_file():
                print(f.relative_to(data))
    elif p.is_file():
        print(small)
EOF
nbin=$(grep -c '^trials/.*\.bin$' "$list" || true)
nimg=$(grep -c '^trials/.*\.img$' "$list" || true)
echo "backing up $nbin .bin and $nimg .img files (plus records) from $data"

stamp="$(date +%Y%m%d-%H%M%S)"
keep=(--backup --backup-dir="replaced/$stamp")        # relative to the destination: slosh-data/replaced/<time>/
rsync -aH $dry "${keep[@]}" --files-from="$list" "$data/" "$BACKUP_DISK/slosh-data/"
if [ -n "${BACKUP_OFFSITE:-}" ]; then
  rsync -aH $dry "${keep[@]}" --files-from="$list" "$data/" "$BACKUP_OFFSITE/slosh-data/"
fi
rdir="$BACKUP_DISK/slosh-data/replaced/$stamp"
if [ -d "$rdir" ]; then
  replaced_dir="$rdir"
  replaced_files=$(find "$rdir" -type f | wc -l | tr -d ' ')
  replaced_bin=$(find "$rdir" -type f -name '*.bin' | wc -l | tr -d ' ')
  echo "$replaced_files file(s) had changed since the last backup; the copies they replaced are in $rdir"
  if [ "$replaced_bin" -gt 0 ]; then
    echo "WARNING: $replaced_bin trial log(s) (.bin) differ from their backup copies. Trial files are never edited (A4):" >&2
    echo "compare them with the old copies in $rdir before relying on either." >&2
  fi
fi
echo "backup finished${dry:+ (dry run)}"
write_result true "$nbin" "$nimg" "ok"
