#!/usr/bin/env bash
# What a run of the release workflow builds, from how it was started (updates.md, sections 4.1,
# 4.3 and 5): its first job runs this and the build jobs take its answers.
#
#   a release tag pushed (v0_0_2)   the stable channel's release 0.0.2, published under that tag
#   by hand, channel beta           the beta channel's, published under the tag "beta"
#   by hand, no channel             a build kept as the run's artifacts: no release, and a planner
#                                   that calls itself a development build
#
# Every one of these is started by the owner. There is no schedule and there is to be none: "DO
# NOT DO ANY FORM OF AUTOMATED RELEASE, EVER ... user must always action it" (2026-11-18).
#
# Reads GITHUB_REF_TYPE, GITHUB_REF_NAME, GITHUB_SHA and WANTED (the channel asked for by hand),
# and asks nothing of the network. Writes name=value lines for $GITHUB_OUTPUT on standard output,
# and what it decided, in words, on standard error:
#
#   build     true
#   channel   stable, beta, or empty for a build that is no release
#   version   the release's numbers with points, 0.0.2; a beta's are those of the last release
#             tag before its commit
#   label     what the downloads' names carry: 0_0_2, or beta-<commit> so that one beta's files
#             are never taken for another's
#   deb       the Debian package's version, which may hold no underscore
#   tag       the tag the release is published under
#   built     now, for every platform's build alike
set -euo pipefail
say() { echo "what.sh: $*" >&2; }
numbers() { # a tag's numbers with points, or nothing for a tag that is not numbers
    local text=${1#v}
    text=${text//_/.}
    [[ $text =~ ^[0-9]+(\.[0-9]+)*$ ]] && echo "$text"
}

sha=${GITHUB_SHA:?}
build=true channel= version= label= deb= tag=
if [ "${GITHUB_REF_TYPE:-}" = tag ]; then
    tag=${GITHUB_REF_NAME:?}
    version=$(numbers "$tag") || { say "$tag is not a release tag: v and numbers, v0_0_2"; exit 1; }
    channel=stable label=${tag#v} deb=$version
    say "release $version, the stable channel's, from the tag $tag"
elif [ "${WANTED:-none}" = beta ]; then
    last=$(git describe --tags --abbrev=0 --match 'v[0-9]*' "$sha" 2>/dev/null) ||
        { say "no release tag before $sha: a beta follows a release"; exit 1; }
    version=$(numbers "$last") || { say "$last is not a release tag"; exit 1; }
    channel=beta tag=beta label="beta-${sha:0:7}" deb="$version+beta.${sha:0:7}"
    say "a beta after release $version, from ${sha:0:12}"
else
    say "a build by hand, kept as this run's artifacts: no release"
fi
printf '%s\n' "build=$build" "channel=$channel" "version=$version" "label=$label" "deb=$deb" \
    "tag=$tag" "built=$(date -u +%Y-%m-%dT%H:%M:%SZ)"
