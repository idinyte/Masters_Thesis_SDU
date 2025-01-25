import subprocess
import re
from sys import exit


subprocess.run("pdftotext -layout main.pdf output.txt", shell=True)

with open('output.txt', 'r', encoding='utf-8') as file:
    text = file.read()

start_pattern = r"Introduction\n\n1.1       Background\n\nRecent"
end_pattern = r"Appendix"

start_match = re.search(start_pattern, text)
end_matches = list(re.finditer(end_pattern, text))
end_match = end_matches[-1] if end_matches else None

if start_match and end_match:
    trimmed_text = text[start_match.start():end_match.end()]
else:
    print("Specified patterns not found in the text")
    exit(1)

trimmed_text = re.sub(r'\s+', ' ', trimmed_text)

with open('output2.txt', 'w', encoding='utf-8') as file:
    file.write(trimmed_text)


tex_file_path = r"chapters\frontpage.tex"
character_count = len(trimmed_text)

# Read and update the LaTeX file
with open(tex_file_path, 'r', encoding='utf-8') as file:
    tex_content = file.read()

updated_tex_content = re.sub(
    r"Character count: \d+",
    f"Character count: {character_count}",
    tex_content
)

with open(tex_file_path, 'w', encoding='utf-8') as file:
    file.write(updated_tex_content)

print(f"Character count updated to {character_count}. Need {2400 * 45}. Done {(100 * character_count / (2400 * 45)):.2f} %")