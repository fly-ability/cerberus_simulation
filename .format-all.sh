# /bin/bash

exclude_list="ethzasl_msf"
all_files=$(git ls-files *.h *.hpp *.c *.cpp)

for exclude in $exclude_list
do
    all_files=$(echo "$all_files" | sed "/$exclude/d")
done

echo "$all_files"
clang-format-6.0 -i ${all_files}
