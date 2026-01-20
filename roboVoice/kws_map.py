from rapidfuzz import fuzz, process

# Command to inference output mapping later to send to ROS2
class CommandMapper:
    def __init__(self, synonyms: dict[str, list[str]], score_cutoff: int = 80):
        self.labels = list(synonyms.keys())
        self.corpus = {k: v for k, v in synonyms.items()}
        self.score_cutoff = score_cutoff
        self.flat = [(k, phrase) for k, phrases in self.corpus.items() for phrase in phrases]

    def map_text(self, text: str) -> str | None:
        if not text: return None
        candidates = [p for _, p in self.flat]
        match, score, _ = process.extractOne(text, candidates, scorer=fuzz.WRatio)
        if score < self.score_cutoff:
            return None
        # find label for matched phrase from dict
        for label, phrase in self.flat:
            if phrase == match: return label
        return None
