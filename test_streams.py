from pylsl import resolve_streams

streams = resolve_streams()
for s in streams:
    print(f"Name: {s.name()}, Type: {s.type()}, Channels: {s.channel_count()}")
