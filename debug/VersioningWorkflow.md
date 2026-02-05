# Project Versioning Workflow Guide

## Overview
This project uses a **Git-Tag-driven versioning system** and follows **Semantic Versioning** (SemVer) Rules. 

Version information is automatically extracted from Git tags during the CMake configuration and embedded into the application (About Dialog, logs, and binary metadata).

### Core Principles
1.  **Source of Truth:** Git Tags are the only source for version numbers.
2.  **Semantic Versioning:** We follow the `Major.Minor.Patch` format (e.g., `1.2.0`).
3.  **Automatic Labeling:** 
    *   **Clean Release:** Build created exactly on a tag (e.g., `1.2.0`).
    *   **In-Development:** Build with commits after a tag (e.g., `1.2.0-dev5-gabc123`).

Note: Always use the "Copy to Clipboard" button in the applications "About" dialog box when reporting bugs to the team.

---

## Developer Setup (Required)

**Automatically update tags when pushing code and during pull/fetch**

To ensure tags are always synchronized between your local machine and the server, run these commands once:

```bash
git config --global push.followTags true
git config --global fetch.tags true
```

## Daily Development.

For daily development work and commit as usual. The build system will automatically update the "commit count" and "hash" (e.g., 1.2.0-dev12-g5f2a1). 

No manual editing of CMakeLists.txt is required for daily commits.

## Creating a Release:

**Choose the Right Version Number.**
Before creating a tag, you must determine what the next version should be.

Check the latest Tag by running this command to see the most recent version in the repository:

```bash
git describe --tags --abbrev=0
```

**Follow Semantic Versioning (SemVer) Rules.** 
Based on the latest tag (e.g., v1.2.4), decide which part to increment:

    PATCH (1.2.X): Increment this for bug fixes and minor optimizations.
        Example: 1.2.4 → 1.2.5
    MINOR (1.X.0): Increment this when you add new features that are backward-compatible. Reset Patch to 0.
        Example: 1.2.4 → 1.3.0
    MAJOR (X.0.0): Increment this when you make breaking changes (changing API, removing functions, or major architectural shifts). Reset Minor and Patch to 0.
        Example: 1.2.4 → 2.0.0

**Verification Step:** If you are unsure, check the list of all recent tags to avoid duplicates:        

```bash
git tag -l -n1 --sort=-v:refname | head -n 10
```

**Create an annotated tag and push to the server:**

```bash
git tag -a v1.2.5 -m "Release version 1.2.5"
git push
```


### Synchronizing

To sync local tags:

```bash
git fetch --all --tags
```

## 🛠 Branching & Versioning Policy (SemVer)
To ensure stability and provide long-term support for older releases, we follow the **Support Branches strategy** combined with Semantic Versioning.

* **Initializing a Support Line.**
When the main (or master) branch reaches a major milestone (e.g., v2.0.0), we create a dedicated support branch for that version.

```bash
	# While on the main branch at the release point
	git tag -a v2.0.0 -m "Release v2.0.0"
	git push origin v2.0.0
	
	# Create a support branch for the version 2.x line
	git checkout -b support/v2.x
	git push origin support/v2.x
```

* **Applying Bugfixes to Older Versions (Patch).**
If a bug is found in v2.0.0 while main has already moved forward to v3.0.0, the fix must be applied to the specific support branch:

```bash
	# Switch to the required support branch
	git checkout support/v2.x
	
	# Apply the fix, test, and commit
	git add .
	git commit -m "fix: resolve critical parsing error"
	
	# Create a new Patch tag
	git tag -a v2.0.1 -m "Release v2.0.1"
	git push origin support/v2.x --tags
```

* **Syncing Fixes to the Main Branch.**
To prevent the bug from reappearing in future releases, "port" the fix from the support branch back to main.

```bash
	# Switch to the main development branch
	git checkout main
	
	# Copy the specific fix using its commit hash
	git cherry-pick <commit_hash_from_support_branch>
	
	# Resolve any conflicts and push
	git push origin main
```

### 📌 Branching Cheat Sheet

**main / master**	: New features, breaking changes MAJOR or MINOR

**support/v1.x**	: Bugfixes for v1 line only	1.x.PATCH

**support/v2.x**	: Bugfixes for v2 line only	2.x.PATCH

**Core Rules:**

Never merge main into a support branch. This would accidentally introduce new features into a stable/patch release.

Always cherry-pick critical bugfixes from support branches back into main.

The Version in main must always be higher than the versions in any support branches.


## Troubleshooting

**Version is "Unknown":**  

Run ```git fetch --tags```


**Version didn't update:**  

Re-run ```cmake ..``` after tagging.

**Inconsistent versions:**  

Run ```git fetch --all --tags --prune```
.
