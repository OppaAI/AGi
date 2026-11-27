# Ministral chat template

`mistral-v7-tekken.jinja` is the unmodified official template from
[`mistralai/Ministral-3-8B-Instruct-2512`](https://huggingface.co/mistralai/Ministral-3-8B-Instruct-2512/blob/d2ff65a33fc5f72d77a65a2bc366feefd44ca320/chat_template.jinja),
revision `d2ff65a33fc5f72d77a65a2bc366feefd44ca320`.
SHA-256: `338c134a61cf54e1d250a8f6c2e974c788c14ae97dd6e7b2491d5446ec472490`.
It is licensed under Apache-2.0 by Mistral AI; see `LICENSE-Ministral-3.txt`.

With `jinja = on`, llama.cpp b12359 treats `chat-template` as Jinja source,
so a built-in name such as `mistral-v7-tekken` becomes literal prompt text.
The router instead uses `chat-template-file` to load this template. Its absolute
path follows the model path in `../llama-router.ini`; update both when moving
the checkout. Keep Jinja enabled for tool definitions, calls and results.

The template uses Mistral V7-Tekken markers, including `[SYSTEM_PROMPT]`,
`[INST]`, `[AVAILABLE_TOOLS]`, `[TOOL_CALLS]`, `[ARGS]` and `[TOOL_RESULTS]`.
Pass AIVA's explicit system message: without one, the upstream template inserts
Mistral's default Le Chat identity. This is a formatting override for the current
Ministral preset and a candidate Nymphaea-RP model; it does not change the model
file, sampling parameters, or make tools available by itself.

## Rendering regression check

Build llama.cpp's CPU-only `test-chat-template` target at `b12359`, then run:

```sh
git clone --depth 1 --branch b12359 https://github.com/ggml-org/llama.cpp.git /tmp/llama-template-check
cmake -S /tmp/llama-template-check -B /tmp/llama-template-check/build \
  -DGGML_CUDA=OFF -DLLAMA_BUILD_TESTS=ON -DLLAMA_BUILD_TOOLS=OFF
cmake --build /tmp/llama-template-check/build --target test-chat-template -j 4
python models/templates/check_template.py /tmp/llama-template-check/build/bin/test-chat-template
```

This checks the router setting, pinned template hash, system/user and multi-turn
content, tool schemas/calls/results, and rejection of an empty conversation
using llama.cpp's actual common chat renderer. No GGUF is needed. Loading the
chosen GGUF and checking a live chat/tool round trip remain deployment checks.
