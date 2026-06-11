# GUIDES

Check `/home/hieu2/gp4_ws/.codex/AGENTS.md`.
For robot catalog material and official Yaskawa reference repositories, check `/home/hieu2/gp4_ws/references/`.

## 1. `.codex` Workspace Metadata
The `.codex/` folder is reserved for Codex-specific workspace guidance and reusable agent instructions, not ROS2 runtime packages.

- `.codex/AGENTS.md` is the folder-local instruction file for anything under `.codex/` and may refine this root policy for documentation and meta-configuration work.
- Use `.codex/agents/` for reusable agent role briefs or delegation prompts tailored to this workspace.
- Use `.codex/skills/` for reusable Codex workflows. Recommended layout: `.codex/skills/<skill_name>/SKILL.md` with optional `references/`, `scripts/`, or `assets/` only when they add real value.
- Use `.codex/rules/` for focused guardrails, conventions, or checklists that are narrower than this repo-wide `AGENTS.md`. Keep one topic per rule file.
- When a `.codex` skill or rule references repo behavior, verify the package path, topic, service, launch file, and command against the actual workspace first.
- If a change introduces a new repo-wide expectation, update both this root `AGENTS.md` and the relevant `.codex` file so instructions do not drift.
- `.codex` content must stay concise, auditable, and aligned with the real workspace structure.
## 2. Adding requires
Required harness for every code wave:

Universal coding harness:

Before modifying files:
1. Print current branch:
   git branch --show-current

2. Inspect packages:
   colcon list

3. Inspect changed files:
   git status --short

During implementation:
- Make the smallest safe change for this wave only.
- Do not implement future wave functionality.
- Do not delete old packages unless this wave explicitly says to.
- Do not call real hardware unless this wave explicitly says hardware execution is allowed.
- Do not add secrets/API keys.
- Do not modify .env except .env.example.
- Add clear logs for every safety rejection.
- Add tests for every safety-critical function.
- Prefer fail-closed behavior.

After modifying files:
1. Build:
   colcon build --symlink-install

2. Run tests:
   colcon test

3. Show test results:
   colcon test-result --verbose

4. Show changed files:
   git status --short

5. Return report in this exact format:
## Wave Report

#Wave ID
...

#Goal
...

#Files Changed
...

#Commands Ran
```bash

```
#Recommend 
- Suggest user what should do next phase/turn.
```

<!-- gitnexus:start -->
# GitNexus — Code Intelligence

This project is indexed by GitNexus as **gp4_ws** (56238 symbols, 98995 relationships, 300 execution flows). Use the GitNexus MCP tools to understand code, assess impact, and navigate safely.

> If any GitNexus tool warns the index is stale, run `npx gitnexus analyze` in terminal first.

## Always Do

- **MUST run impact analysis before editing any symbol.** Before modifying a function, class, or method, run `gitnexus_impact({target: "symbolName", direction: "upstream"})` and report the blast radius (direct callers, affected processes, risk level) to the user.
- **MUST run `gitnexus_detect_changes()` before committing** to verify your changes only affect expected symbols and execution flows.
- **MUST warn the user** if impact analysis returns HIGH or CRITICAL risk before proceeding with edits.
- When exploring unfamiliar code, use `gitnexus_query({query: "concept"})` to find execution flows instead of grepping. It returns process-grouped results ranked by relevance.
- When you need full context on a specific symbol — callers, callees, which execution flows it participates in — use `gitnexus_context({name: "symbolName"})`.

## Never Do

- NEVER edit a function, class, or method without first running `gitnexus_impact` on it.
- NEVER ignore HIGH or CRITICAL risk warnings from impact analysis.
- NEVER rename symbols with find-and-replace — use `gitnexus_rename` which understands the call graph.
- NEVER commit changes without running `gitnexus_detect_changes()` to check affected scope.

## Resources

| Resource | Use for |
|----------|---------|
| `gitnexus://repo/gp4_ws/context` | Codebase overview, check index freshness |
| `gitnexus://repo/gp4_ws/clusters` | All functional areas |
| `gitnexus://repo/gp4_ws/processes` | All execution flows |
| `gitnexus://repo/gp4_ws/process/{name}` | Step-by-step execution trace |

## CLI

| Task | Read this skill file |
|------|---------------------|
| Understand architecture / "How does X work?" | `.claude/skills/gitnexus/gitnexus-exploring/SKILL.md` |
| Blast radius / "What breaks if I change X?" | `.claude/skills/gitnexus/gitnexus-impact-analysis/SKILL.md` |
| Trace bugs / "Why is X failing?" | `.claude/skills/gitnexus/gitnexus-debugging/SKILL.md` |
| Rename / extract / split / refactor | `.claude/skills/gitnexus/gitnexus-refactoring/SKILL.md` |
| Tools, resources, schema reference | `.claude/skills/gitnexus/gitnexus-guide/SKILL.md` |
| Index, status, clean, wiki CLI commands | `.claude/skills/gitnexus/gitnexus-cli/SKILL.md` |

<!-- gitnexus:end -->
