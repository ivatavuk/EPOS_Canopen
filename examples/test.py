import sys
sys.path.append('../../')
from epos import Epos

def main():
    if (sys.version_info < (3, 0)):
        print("Please use python version 3")
        return
    parser = argparse.ArgumentParser(add_help=True,
                                     description='Test Epos CANopen Communication')
    parser.add_argument('--channel', '-c', action='store', default='can0',
                        type=str, help='Channel to be used', dest='channel')
    parser.add_argument('--bus', '-b', action='store',
                        default='socketcan', type=str, help='Bus type', dest='bus')
    parser.add_argument('--rate', '-r', action='store', default=125000,
                        type=int, help='bitrate, if applicable', dest='bitrate')
    parser.add_argument('--nodeID', action='store', default=2, type=int,
                        help='Node ID [ must be between 1- 127]', dest='nodeID')
    parser.add_argument('--objDict', action='store', default='../maxon-70_10.eds',
                        type=str, help='Object dictionary file', dest='objDict')
    args = parser.parse_args()
    
    network = canopen.Network()
    network.connect(channel=args.channel, bustype=args.bus)

    epos = Epos(_network=network)
    if not (epos.begin(args.nodeID, object_dictionary=args.objDict)):
        logging.info('Failed to begin connection with EPOS device')
        logging.info('Exiting now')
        return

    # emcy messages handles
    epos.node.emcy.add_callback(gotMessage)

    # get current state of epos
    state = epos.check_state()
    if state is -1:
        logging.info('[Epos:{0}] Error: Unknown state\n'.format(
            sys._getframe().f_code.co_name))
        return

    if state is 11:
        # perform fault reset
        ok = epos.change_state('fault reset')
        if not ok:
            logging.info('[Epos:{0}] Error: Failed to change state to fault reset\n'.format(
                sys._getframe().f_code.co_name))
            return

    # shutdown
    if not epos.change_state('shutdown'):
        logging.info('Failed to change Epos state to shutdown')
        return
    # switch on
    if not epos.change_state('switch on'):
        logging.info('Failed to change Epos state to switch on')
        return
    if not epos.change_state('enable operation'):
        logging.info('Failed to change Epos state to enable operation')
        return

if __name__ == '__main__':
    main()
