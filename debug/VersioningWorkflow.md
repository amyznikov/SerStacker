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


## Branching & Hotfixes

* Feature Branches.

 Feature Branches inherit the last tag from 'main' plus the commit count. 
 
 This helps identify the branch during testing.


* Hotfix Procedure

If a bug is found in some version, for example v1.2.0:

```bash
	# Checkout from tag: 
	git checkout -b hotfix/name v1.2.0
	
	# Apply fix and commit.
	# Tag new version: 
	git tag -a v1.2.1 -m "Hotfix description"
	
	# Merge back to main and push tags: 
	git push origin --tags
```


## Troubleshooting

**Version is "Unknown":**  

Run ```git fetch --tags```


**Version didn't update:**  

Re-run ```cmake ..``` after tagging.

**Inconsistent versions:**  

Run ```git fetch --all --tags --prune```
.
