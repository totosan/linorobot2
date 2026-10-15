---
name: skill-maintenance
description: "Review and maintain workspace Copilot skills and related instructions from concrete project changes or chat lessons. Use for skill maintenance, update skills, stale guidance, new reusable workflow, recurring blockers, or a periodic customization audit."
user-invocable: true
---

# Workspace skill maintenance

Maintain the project's Copilot skills and related agent customizations from
verified evidence. This is an on-demand workflow, not a background scheduler.
Invoke it after a meaningful workflow change, when a chat reveals a recurring
failure or reusable procedure, or during a periodic project-maintenance review.

## Procedure

1. **Establish scope and current state.** Confirm whether the work is workspace-
   or user-profile-wide if the user has not already specified it. For this
   project, inspect `.github/skills/`, `.github/instructions/`, and only the
   related prompts/agents when a specific change points to them. Check the
   relevant Git status before editing; preserve existing and untracked changes.
2. **Extract lessons, not chronology.** Use this chat and the current source,
   tests, and project docs to identify stable instructions: recurring decisions,
   verified commands, failure modes, safety boundaries, and current workflow
   ownership. Separate durable procedure from date-sensitive status, one-off
   failures, assumptions, and unverified claims. Keep run-specific evidence in
   the project's status docs rather than copying it into a skill.
3. **Find the right existing home.** Read the full candidate skill/instruction
   and nearby references. Update an existing customization when the lesson
   belongs to its current concern. Create a new skill only for a distinct,
   repeatable, multi-step workflow that does not fit existing guidance. Do not
   duplicate instructions across files; add a short link when another file is
   authoritative.
4. **Surface blockers clearly.** When missing packages, tools, config files,
   installation artifacts, permissions, or runtime prerequisites block progress,
   report all known missing items, their expected locations or versions, the
   affected step, and the evidence for the blocker. Recommend a minimal repair
   path, state its side effects and target environment, and ask whether to
   perform the installation when possible. Never treat a source file as proof
   that the intended build/install overlay contains it. Verify both the installed
   artifact and the dependent configuration or behavior after an authorized
   repair. A failed runtime operation is not permission for an automatic retry.
5. **Make a precise edit.** Preserve unrelated guidance and user changes. Prefer
   changing the smallest relevant section; avoid broad rewrites, speculative
   setup steps, redundant checklists, and date-stamped run history in reusable
   skills. If a change would alter safety behavior, access permissions, external
   service use, hardware behavior, or trigger consequential installation, get
   explicit user direction before proceeding.
6. **Validate customizations.** Confirm each skill folder name matches the
   frontmatter `name`; required `description` is clear, keyword-rich, and under
   the platform limit; YAML frontmatter parses; referenced relative files exist;
   commands and paths match the current workspace layout; and updated guidance
   does not contradict project status or another customization. Run focused
   tests or syntax checks only when the edited customization contains executable
   commands or the repository has a relevant validator.
7. **Report the maintenance result.** Summarize which files were updated or
   added, the durable lesson each captures, validation performed, and any
   remaining uncertainties or user-authorized actions. Do not claim skills run
   periodically by themselves: recommend invoking `/skill-maintenance` after
   major workflow changes or during a chosen review cadence.

## Project-specific Nav2 replay lessons

- A local workspace source tree and the OrbStack Humble install overlay are
  different states. Discover the actual colcon base path and explicitly target
  build/install/log paths before building; verify the installed share files
  afterward.
- The Fortress Nav2 profile depends on the shared `navigation_common.yaml`.
  Compose the common and profile files through the existing navigation helper;
  do not pass only the Fortress fragment to Nav2.
- In the Humble CLI, `ros2 lifecycle get NODE` reports a state such as
  `active [3]`; validate active lifecycle state, not merely node existence.
- For replay acceptance, verify received messages and costmap outputs as well as
  bag metadata, and check owned-process and graph cleanup. A transport/costmap
  replay without a goal does not constitute a G4/G5 navigation re-run.
- On any failed runtime attempt, inspect its full logs and cleanup state, update
  project evidence, explain the blocker and next safe action, and wait for fresh
  approval before retrying.
