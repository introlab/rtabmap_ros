import shlex


def split_launch_arguments(argument_string):
    if not argument_string:
        return []
    return shlex.split(argument_string)


def filter_rtabmap_arguments(argument_string):
    kept = []
    removed = []
    tokens = split_launch_arguments(argument_string)

    index = 0
    while index < len(tokens):
        token = tokens[index]
        if token.startswith("--Odom/"):
            removed.append(token)
            if "=" not in token and index + 1 < len(tokens):
                next_token = tokens[index + 1]
                if not next_token.startswith("--"):
                    removed.append(next_token)
                    index += 2
                    continue
            index += 1
            continue

        kept.append(token)
        index += 1

    return kept, removed
