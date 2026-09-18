import json
from pathlib import Path

Import("env")

project_dir = Path(env.subst("$PROJECT_DIR"))
source_path = project_dir / "debug_config.json"
output_path = project_dir / "include" / "debug_config.generated.h"

# Fail the build early with a useful location if somebody edits invalid JSON.
config = json.loads(source_path.read_text(encoding="utf-8"))
compact = json.dumps(config, separators=(",", ":"), ensure_ascii=True)

generated = (
    "#pragma once\n\n"
    "namespace EmbeddedDebugConfig {\n"
    f'inline constexpr char JSON[] = R"DEBUG_JSON({compact})DEBUG_JSON";\n'
    "}\n"
)

if not output_path.exists() or output_path.read_text(encoding="utf-8") != generated:
    output_path.write_text(generated, encoding="utf-8")
