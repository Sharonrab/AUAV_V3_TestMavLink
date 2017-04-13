"""
This script will copy a log file to a new file, removing headers and footers along the way.
Run in command line: "python3 dataExtract.py LOG_FILE [LOG_FILE2 ...]"
Example: "python3 dataExtract.py 0001.log"
"""

import sys

def checkSum(data):
    """Calculates the XOR checksum of the input."""
    cs = 0
    for n in data:
        cs = cs ^ n
    return cs

# Process input arguments
if len(sys.argv) < 2 :
    print("This script requires at least 1 argument: a logger file to extract from.")
    exit()

# Set some constants describing the file format
HEADER_TAG_LEN = 2
HEADER_LEN = HEADER_TAG_LEN + 1
FOOTER_TAG_LEN = 2
FOOTER_LEN = FOOTER_TAG_LEN + 1
FILE_NUMBER_POS = 2
CHECKSUM_POS = -3

# Open files
for f in sys.argv[1:]:
    f_out = f.replace('.log', '_extracted.log')
    print("Decoding {} into {}.".format(f, f_out))
    with open(f, "rb") as slog, open(f_out, "wb") as outlog:

        # Main loop, reading file by sectors and processing each of them.
        nextSector = slog.read(512)
        nextSector = bytearray(nextSector) # Always cast to a bytearray so this is Python2 & 3 compatible
        fileNumber = nextSector[FILE_NUMBER_POS]
        sectorNumber = 0
        error = False
        while nextSector:
            # Verify header & footer tags (Errors here may signify dataloss)
            if nextSector[:HEADER_TAG_LEN] != b'%^':
                print("ERROR: Header tag failed at sector {}.".format(sectorNumber))
                error = True
                break
            if nextSector[-FOOTER_TAG_LEN:] != b'%$':
                print("ERROR: Footer tag failed at sector {}.".format(sectorNumber))
                error = True
                break

            # Verify header number (Errors here are generally not indicative of data loss)
            if nextSector[FILE_NUMBER_POS] != fileNumber:
                print("WARNING: File number mismatch at sector {}. This is likely not an error, as it usually happens when files don't end on a 128kiB boundary. Please verify that all data is intact if possible.".format(sectorNumber))
                error = True
                break

            # Verify footer checksum (Errors here may signify dataloss)
            if nextSector[CHECKSUM_POS] != checkSum(nextSector[HEADER_LEN:-FOOTER_LEN]):
                print("ERROR: Checksum failed at sector {}.".format(sectorNumber))
                error = True
                break

            # Output the actual sector data and prepare for next iteration
            outlog.write(nextSector[HEADER_LEN:-FOOTER_LEN])
            sectorNumber += 1
            nextSector = slog.read(512)
            nextSector = bytearray(nextSector) # Always cast to a bytearray so this is Python2 & 3 compatible

        print("Decoded {} bytes into '{}'.".format((sectorNumber + 1) * 506, f_out))
