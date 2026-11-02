import os

# Files shared between nodes (maze file, robot track, saved maps).
# Override with SEARCHFORCE_CACHE; created on import.
CACHE_DIR = os.path.expanduser(os.environ.get('SEARCHFORCE_CACHE', '~/searchforce_cache'))
os.makedirs(CACHE_DIR, exist_ok=True)
