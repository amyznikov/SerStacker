#pragma once
#ifndef __project_version_h__
#define __project_version_h__

#define PROJECT_VERSION_MAJOR 1
#define PROJECT_VERSION_MINOR 1
#define PROJECT_VERSION_PATCH 2
#define PROJECT_GIT_HASH     "10dd48d"

#define PROJECT_VERSION_INT(major, minor, patch) ((major << 16) | (minor << 8) | patch)
#define PROJECT_VERSION_CURR PROJECT_VERSION_INT(PROJECT_VERSION_MAJOR, PROJECT_VERSION_MINOR, PROJECT_VERSION_PATCH)

#define PROJECT_VERSION "1.1.2"
#define PROJECT_FULL_VERSION_STR "1.1.2-10dd48d"

#endif // __project_version_h__

