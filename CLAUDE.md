# Project context

This is my ai agent workspace I am compieting in a Tech compatition my idea for winning is making a "Mind controlled " electric wheelchair, Im only gona make a prototype for now using the Muse 2 EEG , This laptop -> later gonna use a Raspberry pi , and an Arduino car as a simulator for the wheelchair bcz they are in theory THE same.

# Rules

-Tell me the aproximet amount of tokens u use before starting
-Wait until i allow you to continue or not with the task
-Tell me if you can reduce the amount of tokens needed
-Always ask clarifying questions before starting a complex task
-keep reports and summeries concise - bulletpoints over paragraphs
-Create agents if you see that the task is fitting for an agent to do the best task possible
-Ask before creating an agent and tell me how many agents i have in total
-Add everything to my github reposetry
-Always use a qualityofcode checker agent

# Project Structure

-Workflow /- workflow instructions files(plain englisch reciets that the agent follows )
-output / - Finisched delivrables (report , draft analysis )

# Agent Orchestration

- For any task that involves more than one concern (coding, testing, analysis, hardware, documentation, etc.), always use the `projectplanner` agent to decompose and coordinate the
  work
- The `projectplanner` agent lives at `~/.claude/agents/projectplanner.md` and is always available
- Never start a multi-part task without first routing it through `projectplanner`
- `projectplanner` will find existing agents before creating new ones — do not create agents manually without checking with it first
- All task results must be reported back in the structured bullet-point format that `projectplanner` produces

Add this as a new section in your existing CLAUDE.md, right after the # Rules section. It tells Claude to always route complex tasks through projectplanner and reinforces its role
as the single entry point for multi-part work.

## graphify

This project has a knowledge graph at graphify-out/ with god nodes, community structure, and cross-file relationships.

Rules:
- For codebase questions, first run `graphify query "<question>"` when graphify-out/graph.json exists. Use `graphify path "<A>" "<B>"` for relationships and `graphify explain "<concept>"` for focused concepts. These return a scoped subgraph, usually much smaller than GRAPH_REPORT.md or raw grep output.
- If graphify-out/wiki/index.md exists, use it for broad navigation instead of raw source browsing.
- Read graphify-out/GRAPH_REPORT.md only for broad architecture review or when query/path/explain do not surface enough context.
- After modifying code, run `graphify update .` to keep the graph current (AST-only, no API cost).
