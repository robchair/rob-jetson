from brainflow.board_shim import BoardShim, BrainFlowInputParams, BoardIds
import time

params = BrainFlowInputParams()
board = BoardShim(BoardIds.MUSE_2_BOARD, params)

board.prepare_session()
board.start_stream()

time.sleep(5)
data = board.get_board_data()

print("Shape:", data.shape)
print("\nBoard description:")
print(BoardShim.get_board_descr(BoardIds.MUSE_2_BOARD))

print("\nEEG channels:", BoardShim.get_eeg_channels(BoardIds.MUSE_2_BOARD))

# Print all 8 channels so we can see what's there
for i in range(data.shape[0]):
    print(f"Channel {i}: min={data[i].min():.2f} max={data[i].max():.2f} mean={data[i].mean():.2f}")

board.stop_stream()
board.release_session()
