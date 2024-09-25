import itertools

def solve_cryptarithm(words, result):
    unique_letters = ''.join(set(''.join(words) + result))
    
    if len(unique_letters) > 10:
        print("Too many unique letters for a single-digit solution.")
        return
    
    digits = '0123456789'
    
    for perm in itertools.permutations(digits, len(unique_letters)):
        letter_to_digit = dict(zip(unique_letters, perm))
        
        def word_to_number(word):
            return int(''.join(letter_to_digit[letter] for letter in word))
        
        if any(letter_to_digit[word[0]] == '0' for word in words + [result]):
            continue
        
        sum_words = sum(word_to_number(word) for word in words)
        
        if sum_words == word_to_number(result):
            print("Solution found!")
            for word in words:
                print(f"{word} = {word_to_number(word)}")
            print(f"{result} = {word_to_number(result)}")
            print(f"Letter to Digit Mapping: {letter_to_digit}")
            return
    
    print("No solution found.")

input_words = input("Enter the words to sum (space-separated): ").upper().split()
input_result = input("Enter the result word: ").upper()

solve_cryptarithm(input_words, input_result)
