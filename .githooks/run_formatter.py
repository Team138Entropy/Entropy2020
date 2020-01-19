import os
from pathlib import Path

jar_file = ".format/google-java-format-1.7-all-deps.jar"
java_files = list(map(lambda s: str(s), Path("src/main/java").rglob("*.java")))

exit(os.system(f"java -jar {jar_file} --replace {' '.join(java_files)}"))
