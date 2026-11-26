#!/usr/bin/env bash

readonly SUBJECT_PATTERN='^(feat|fix|docs|style|refactor|perf|test|build|ci|chore|revert)(\([a-z0-9][a-z0-9._/-]*\))?!?: .+$'
COMMIT_MESSAGE_ERRORS=()

validate_commit_message() {
    local message=${1//$'\r\n'/$'\n'}
    message=${message//$'\r'/$'\n'}

    local -a lines=()
    mapfile -t lines <<< "$message"
    local subject=${lines[0]}
    COMMIT_MESSAGE_ERRORS=()

    if [[ $subject == Merge\ * || $subject == Revert\ \"* ]]; then
        return 0
    fi

    if [[ ! $subject =~ $SUBJECT_PATTERN ]]; then
        COMMIT_MESSAGE_ERRORS+=(
            "subject must match: <type>(<optional scope>): <summary>; allowed types: feat, fix, docs, style, refactor, perf, test, build, ci, chore, revert"
        )
    fi
    if (( ${#subject} > 72 )); then
        COMMIT_MESSAGE_ERRORS+=("subject is ${#subject} characters; maximum is 72")
    fi
    if [[ $subject =~ ^[[:space:]] || $subject =~ [[:space:]]$ ]]; then
        COMMIT_MESSAGE_ERRORS+=("subject must not have leading/trailing whitespace")
    fi
    if (( ${#lines[@]} > 1 )) && [[ ${lines[1]} =~ [^[:space:]] ]]; then
        COMMIT_MESSAGE_ERRORS+=("subject and body must be separated by a blank line")
    fi

    local index line number
    for index in "${!lines[@]}"; do
        line=${lines[index]}
        number=$((index + 1))
        if [[ $line =~ [[:space:]]$ ]]; then
            COMMIT_MESSAGE_ERRORS+=("line ${number} has trailing whitespace")
        fi
        if (( index >= 2 && ${#line} > 100 )) &&
            [[ $line != *http://* && $line != *https://* ]]; then
            COMMIT_MESSAGE_ERRORS+=("body line ${number} exceeds 100 characters")
        fi
    done
}

print_commit_message_errors() {
    local error
    for error in "${COMMIT_MESSAGE_ERRORS[@]}"; do
        printf '  - %s\n' "$error" >&2
    done
}
