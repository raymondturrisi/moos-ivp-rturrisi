#!/bin/bash

function kill_descendants {
	local pid=$1

	local children=$(pgrep -P $pid)
	for child in $children; do
		kill_descendants $child
	done

	kill $pid 2> /dev/null
}

kill_descendants $1
