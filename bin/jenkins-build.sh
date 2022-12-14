#!/bin/bash

# format seconds in a more readable way
format_time() {
	local t=$1
	printf "%02d:%02d:%02ds" $((t/3600)) $(( (t%3600)/60 )) $((t%60))
}

# we need our tools
export PATH="/usr/local/bin:$PATH"
# do not install anything into /tftpboot
export NO_BUILD_INTO_TFTPBOOT=1
# one date for all builds
export BUILD_START_STRING=$(date -u -R)
# we are the build machine,  no hashes in version
export ON_BUILD_MACHINE=1

TARGETS=
#TARGETS="$TARGETS AcceleratedConcepts/5300-DC"
TARGETS="$TARGETS AcceleratedConcepts/5301-DC"
TARGETS="$TARGETS AcceleratedConcepts/5400-RM"
TARGETS="$TARGETS AcceleratedConcepts/5400-LX"
TARGETS="$TARGETS AcceleratedConcepts/6300-CX"
TARGETS="$TARGETS AcceleratedConcepts/6350-SR"
TARGETS="$TARGETS AcceleratedConcepts/6300-LX"
TARGETS="$TARGETS AcceleratedConcepts/8300"
TARGETS="$TARGETS AcceleratedConcepts/Factory8200"
TARGETS="$TARGETS AcceleratedConcepts/9400-UA"

# clean out releases for each build
rm -rf release

START_BUILD=$(date +%s)

PASS=0
FAIL=0
PASS_LIST=
for target in $TARGETS; do
    echo "BUILDING: ${target}"
    # make sure we are truly clean before we start
    make distclean > /dev/null 2>&1
    START_TARGET=$(date +%s)
    make ${target}_default && make release
    rc=$?
    END_TARGET=$(date +%s)
    elapsed=$((END_TARGET - START_TARGET))
    eval ELAPSED_$(echo -n $target | tr -c 'a-zA-Z0-9_' '_')=$elapsed
    if [ $rc -eq 0 ]; then
        echo "INFO: ${target} completed in $(format_time $elapsed)"
        PASS=$((PASS + 1))
        PASS_LIST="$PASS_LIST:$target:"
    else
        echo "ERROR: ${target} failed in $(format_time $elapsed)"
        FAIL=$((FAIL + 1))
    fi
    # make sure we are truly clean when we end
    make distclean > /dev/null 2>&1
done

echo "BUILDING: results"
for target in $TARGETS; do
    eval elapsed=\$ELAPSED_$(echo -n $target | tr -c 'a-zA-Z0-9_' '_')
    case "$PASS_LIST" in
    *:$target:*) echo "PASS: $target in $(format_time $elapsed)" ;;
    *)           echo "FAIL: $target in $(format_time $elapsed)" ;;
    esac
done

END_BUILD=$(date +%s)

echo "---------------------------------------------"
echo "$PASS builds passed, $FAIL builds FAILED, $(format_time $((END_BUILD - START_BUILD)) ) build time."
echo "---------------------------------------------"

exit $FAIL
