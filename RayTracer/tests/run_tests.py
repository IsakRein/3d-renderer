import os
import subprocess 

inputs = []
answers = []

for file in os.listdir('./tests'):
    if file.endswith('.in'):
        inputs.append(file)
    elif file.endswith('.ans'):
        answers.append(file)

inputs.sort()
answers.sort()
assert list(map(lambda x: x.replace('.in', ''), inputs)) == list(map(lambda x: x.replace('.ans', ''), answers))

inputs = list(map(lambda x: './tests/' + x, inputs))
answers = list(map(lambda x: './tests/' + x, answers))

test_cases = zip(inputs, answers)
success_count = 0

for input_file, answer_file in test_cases:
    test_name = input_file.replace('./tests/', '').replace('.in', '')
    with open(input_file, 'r') as file:
        input_data = file.read()
    with open(answer_file, 'r') as file:
        answer_data = file.read()
    process = subprocess.run(["build/main"], input=input_data.encode(), capture_output=True)
    result = process.stdout.decode()
    if result == answer_data:
        print(f"🟩 Passed test: {test_name}")
        success_count += 1
    else:
        print(f"🟥 Failed test: {test_name}")
        print("\nExpected:")
        print(answer_data)
        print("\nActual:")
        print(result)

print("\n-----------------")
print(f"Passed {success_count}/{len(inputs)} tests!\n\n")