from pynput import keyboard

while True:
    with keyboard.Events() as events:
        # Block at most one second
        event = events.get(1.0)
        if event is None:
            print('You did not press a key within one second')
        else:
            print('Received event {}'.format(event))
            if event.key.char == 'f':
                print('worked')

