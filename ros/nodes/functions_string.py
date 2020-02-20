""" Simple functions to manipulate strings """

# Replace the functions below with your implementation as described in the assignment


def is_rhyme(word1, word2, k):
    """
    Returns True if the last k letters of the two words are the same
    (case sensitive). Automatically returns False if either word contains less than k letters.
    """
    endWord1 = word1[-k:]
    endWord2 = word2[-k:]

    if (len(word1)<k) or (len(word2)<k):
        return False

    elif endWord1 == endWord2:
        return True
    else:
        return False
