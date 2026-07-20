# Copilot instructions for architecture study

The goal is to study architecture and produce evidence-backed documentation.

Rules:
- Do not modify source code unless explicitly asked.
- Prefer read-only analysis.
- Always cite file paths, function names, and classes.
- Separate facts from interpretation.
- Mark uncertain claims as TODO or hypothesis.
- For diagrams, use Mermaid.
- For tables, use Markdown tables.
- Ignore build/, install/, external build outputs, generated files, and binary files.
- Focus on SEAHOWL ↔ Chrono ↔ OpenFAST module boundaries.
- When comparing to SEA-Stack, separate:
  1. What SEAHOWL already does
  2. What SEA-Stack already does
  3. What can be reused
  4. What needs redesign