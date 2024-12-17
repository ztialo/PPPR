curl -sL "https://keyserver.ubuntu.com/pks/lookup?op=get&search=0x9165938D90FDDD2E" | gpg --dearmor | sudo tee /etc/apt/trusted.gpg.d/raspbian-archive-key.gpg > /dev/null
