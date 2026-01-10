from codecs import charmap_encode
from collections import deque

can_data = deque()

with open("t4.txt", "r") as f:
    for x in f:
        characters = x.split("  ")
        characters_nospace = []

        for i in characters:
            if i != " " and i != "":
                characters_nospace.append(i)

        print(characters_nospace)

        can_data.append(
            [
                bytes.fromhex(characters_nospace[5]),
                bytes.fromhex(characters_nospace[7][0:-1].replace(" ", "")),
            ]
        )

print(can_data)
