# Project Guidelines

## Interaction Rules

- When multiple valid approaches exist, present all options to the user and let them decide before making any changes. Use the interactive `AskUserQuestion` tool with suggested options whenever a decision needs to be made.

## Commit Style

- Use [Conventional Commits](https://www.conventionalcommits.org/) (e.g. `feat:`, `fix:`, `chore:`, `docs:`, `refactor:`, `test:`).
- The first line of the commit message must be all lowercase.
- Do not add Co-Authored-By trailers to commit messages.

## Code Quality

- Keep tabs on compiler warnings and address them immediately. Never let warnings build up.
- Verify changes by running `cargo build` and `cargo clippy` after making code changes.
- Write unit tests for pure logic and utility functions where it makes sense. Hardware-dependent code can only be tested end-to-end.
- When resolving version conflicts, prioritize core crates (e.g. `esp-hal`) over peripheral dependencies. Adjust the less important crate to match, not the other way around.
- Add Rust doc comments (`///`) on public items and inline comments for complex logic.

## Shell Commands

- Never use absolute paths in shell commands. Use plain commands from the working directory (e.g. `git push`, not `git -C /full/path push`).

## Process Management

- Keep track of any application instances you start and ensure only a single instance is running at a time. Kill previous instances before starting new ones.

## Versioning

- Follow [Semantic Versioning](https://semver.org/) for `Cargo.toml` version bumps.
- Suggest a version bump when a meaningful milestone is reached (new feature, breaking change, significant fix). Don't bump for every small commit — group related changes under one version when possible.

## Changelog

- Track changes in `CHANGELOG.md` using [Keep a Changelog](https://keepachangelog.com/) format.
- Update the changelog as part of each commit when applicable.
