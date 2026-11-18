#!/usr/bin/env bash
# What a run of the release workflow builds, from how it was started (updates.md, sections 4.1,
# 4.3 and 5): its first job runs this and the build jobs take its answers.
#
#   a release tag pushed (v0_0_2)   the stable channel's release 0.0.2, published under that tag
#   the nightly schedule            the beta channel's, published under the tag "beta" - but only
#                                   when main has moved since the last beta and its CI passed
#   by hand, channel beta           the beta channel's, whatever the last one was
#   by hand, no channel             a build kept as the run's artifacts: no release, and a planner
#                                   that calls itself a development build
#
# Reads GITHUB_REF_TYPE, GITHUB_REF_NAME, GITHUB_EVENT_NAME, GITHUB_SHA, GITHUB_SERVER_URL,
# GITHUB_REPOSITORY and WANTED (the channel asked for by hand); the nightly check asks GitHub with
# curl and gh. Writes name=value lines for $GITHUB_OUTPUT on standard output, and what it decided,
# in words, on standard error:
#
#   build     true, or false for a night with nothing new
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
elif [ "${GITHUB_EVENT_NAME:-}" = schedule ] || [ "${WANTED:-none}" = beta ]; then
    last=$(git describe --tags --abbrev=0 --match 'v[0-9]*' "$sha" 2>/dev/null) ||
        { say "no release tag before $sha: a beta follows a release"; exit 1; }
    version=$(numbers "$last") || { say "$last is not a release tag"; exit 1; }
    channel=beta tag=beta label="beta-${sha:0:7}" deb="$version+beta.${sha:0:7}"
    say "a beta after release $version, from ${sha:0:12}"
    if [ "${GITHUB_EVENT_NAME:-}" = schedule ]; then
        # The nightly one: not when the beta already is this commit, nor before CI has passed it.
        manifest="${GITHUB_SERVER_URL:?}/${GITHUB_REPOSITORY:?}/releases/download/beta/update.json"
        current=$(curl -fsSL "$manifest" 2>/dev/null |
            python3 -c 'import json, sys; print(json.load(sys.stdin)["commit"])' 2>/dev/null || true)
        if [ "$current" = "$sha" ]; then
            say "the beta is already this commit: nothing to build tonight"
            build=false
        else
            # CI's verdict is on the last commit that changed more than the release notes: the one
            # the release workflow commits after a release is its own, pushed with the workflow's
            # token, which starts no CI.
            code=$(git log -1 --format=%H "$sha" -- . ':(exclude)RELEASE_NOTES.md')
            if [ "$(gh run list --workflow CI --commit "$code" --status success --limit 1 --json databaseId --jq length)" != 1 ]; then
                say "CI has not passed on ${code:0:12}: nothing to build tonight"
                build=false
            fi
        fi
    fi
else
    say "a build by hand, kept as this run's artifacts: no release"
fi
printf '%s\n' "build=$build" "channel=$channel" "version=$version" "label=$label" "deb=$deb" \
    "tag=$tag" "built=$(date -u +%Y-%m-%dT%H:%M:%SZ)"
