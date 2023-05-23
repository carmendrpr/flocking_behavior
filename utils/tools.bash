#!/bin/bash

# Function to convert a list into a comma-separated string
function list_to_string() {
  local IFS=','  # Set the separator as a comma
  echo "$*"
}

# Function to convert a comma-separated string into a list
function string_to_list() {
  IFS=',' read -r -a elements <<< "$1"
  echo "${elements[@]}"
}
