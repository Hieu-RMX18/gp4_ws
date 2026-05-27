# GUIDES

Check `home/hieu2/gp4_ws/.codex/AGENTS.md`
More details about catalog robot and offical repos yaskawa check `home/hieu2/gp4_ws/references/`.

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
