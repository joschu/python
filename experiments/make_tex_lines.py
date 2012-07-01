import string
lines = []
for letter in string.letters:
    lines.append(r"\newcommand{\b%s}{\mathbf{%s}}"%(letter,letter))
other = [
    ("0","zero"),
    ("1","one"),
    ("cdot","dot")
]
for (c,s) in other:
    lines.append(r"\newcommand{\b%s}{\mathbf{%s}}"%(s,c))
    