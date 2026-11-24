"""CLI de preview e navegação individual supervisionada (go exige --confirm)."""
import argparse
import sys
import time

import rclpy
from std_msgs.msg import String


def main():
    parser = argparse.ArgumentParser(description='Gerenciar bancadas e, opcionalmente, navegar com autorizacao explicita.')
    parser.add_argument('command', choices=['arm', 'preview', 'compare', 'between', 'list', 'clear', 'save', 'load', 'go', 'cancel', 'check'])
    parser.add_argument('name', nargs='?', help='Identificador da bancada, ex.: A ou B')
    parser.add_argument('destination', nargs='?', help='Destino (para between A B)')
    parser.add_argument('--confirm', action='store_true', help='Autorizacao para GO fisico: go A --confirm')
    args = parser.parse_args()
    if args.command in ('arm', 'preview', 'clear', 'go', 'check') and not args.name:
        parser.error(f'{args.command} requer identificador, exemplo: {args.command} A')
    if args.command == 'between' and (not args.name or not args.destination):
        parser.error('between requer origem e destino, exemplo: between A B')
    if args.command not in ('arm', 'preview', 'clear', 'between', 'go', 'check') and args.name:
        parser.error(f'{args.command} nao aceita identificador')
    if args.command != 'between' and args.destination:
        parser.error(f'{args.command} nao aceita segundo identificador')
    if args.command == 'go' and not args.confirm:
        parser.error('go precisa de --confirm, exemplo: station_cli go A --confirm')
    if args.command != 'go' and args.confirm:
        parser.error('--confirm permitido somente no comando go')
    rclpy.init()
    node = rclpy.create_node('station_cli')
    pub = node.create_publisher(String, '/station_manager/command', 10)
    try:
        deadline = time.monotonic() + 6.0
        while pub.get_subscription_count() < 1 and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if pub.get_subscription_count() < 1:
            print('ERRO: /station_manager nao esta recebendo comandos. Inicie o launch primeiro.', file=sys.stderr)
            sys.exit(2)
        command = args.command + (' ' + args.name if args.name else '') + (' ' + args.destination if args.destination else '') + (' CONFIRM' if args.confirm else '')
        pub.publish(String(data=command))
        print(f'Enviado: {command}. Veja /station_manager/status ou terminal do station_manager.')
        # Dá tempo para a publicação sair antes de destruir o publisher.
        time.sleep(0.35)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
