import os.path

Import("projenv")

include_flags = []
# print(projenv.keys())
# exit(1)
include_dir: str = projenv["PROJECT_INCLUDE_DIR"]
src_dir: str = projenv["PROJECT_SRC_DIR"]
deps_dir: str = projenv["PROJECT_LIBDEPS_DIR"]

paths: list[str] = projenv["CPPPATH"]
to_remove = []
for i, path in enumerate(paths):
    if not path.startswith(include_dir) and not path.startswith(src_dir): # and not path.startswith(deps_dir):
        # pass
        include_flags.append(f"-isystem")
        include_flags.append(f"\"{path}\"")
        to_remove.append(path)

# print(type(projenv["CXXFLAGS"]))
# print("Build flags0: ", projenv["CXXFLAGS"])

projenv.Append(
    CXXFLAGS = include_flags
)
projenv["CPPPATH"] = list(set(projenv["CPPPATH"]) - set(to_remove))

# print(projenv["CPPPATH"])
# print(projenv["PROJECT_SRC_DIR"])
# print(projenv["PROJECT_SOURCE_DIR"])
# print("Build flags: ", projenv["CXXFLAGS"])
# exit(1)