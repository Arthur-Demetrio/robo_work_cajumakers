# Acesso remoto ao robô (Tailscale + Remote-SSH)

## PC do robô

```bash
sudo apt update
sudo apt install openssh-server
sudo systemctl enable --now ssh

curl -fsSL https://tailscale.com/install.sh | sh
sudo systemctl enable --now tailscaled
sudo tailscale up        # login com conta PESSOAL (não a da escola)

tailscale ip -4          # deve mostrar 100.105.47.35
```

## Segundo PC

```bash
sudo apt install openssh-client

curl -fsSL https://tailscale.com/install.sh | sh
sudo systemctl enable --now tailscaled
sudo tailscale up        # mesma conta pessoal do robô

tailscale status         # o robô deve aparecer na lista
ssh marcos@100.105.47.35  # testar (exit para sair)

ssh-keygen -t ed25519
ssh-copy-id marcos@100.105.47.35
```

`~/.ssh/config`:

```
Host robo
    HostName 100.105.47.35
    User marcos
```

## VS Code (segundo PC)

1. Instalar a extensão **Remote - SSH**
2. `Ctrl+Shift+P` → **Remote-SSH: Connect to Host...** → `robo`
3. **File → Open Folder** → pasta do código no PC do robô (`/home/marcos/...`)

## Se entrou com a conta errada no Tailscale

```bash
sudo tailscale logout
sudo tailscale up --force-reauth
```
