FROM gitpod/workspace-python-3.11:latest

# Install & configure Doppler CLI
RUN (curl -Ls --tlsv1.2 --proto "=https" --retry 3 https://cli.doppler.com/install.sh || wget -t 3 -qO- https://cli.doppler.com/install.sh) | sudo sh

# Next, install tailscale
RUN curl -fsSL https://tailscale.com/install.sh | sh