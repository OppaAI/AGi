# gcc.py (NO ROS, async Python service, NO LLM service)

"""
GCC - Non-blocking Ollama wrapper (NO LLM service)

Runs LLM in background thread, exposes async interface.
Can be used by ROS nodes WITHOUT blocking.
"""

import ollama
import queue
import threading
from dataclasses import dataclass
from typing import Optional, Callable

@dataclass
class GCCRequest:
    """Request to LLM"""
    prompt: str
    callback: Callable[[str], None]
    conversation_id: Optional[str] = None

class GCC:
    """
    GCC - Non-blocking LLM service
    
    Like a specialized brain region:
    - Runs in background (doesn't block)
    - Processes requests from queue
    - Callbacks when response ready
    """
    
    def __init__(self, model_name: str):
        self.model_name = model_name
        self.request_queue = queue.Queue()
        self.conversations = {}  # conversation_id -> messages
        self.worker_thread = None
        self.running = False
    
    def start(self):
        """Start LLM worker thread"""
        if self.running:
            return
        
        self.running = True
        self.worker_thread = threading.Thread(
            target=self._worker_loop,
            daemon=True,
            name="GCC.Worker"
        )
        self.worker_thread.start()
    
    def stop(self):
        """Stop LLM worker thread"""
        self.running = False
        if self.worker_thread:
            self.worker_thread.join(timeout=5.0)
    
    def _worker_loop(self):
        """Background thread: process LLM requests"""
        while self.running:
            try:
                # Wait for request
                request = self.request_queue.get(timeout=1.0)
                
                # Get conversation history
                conv_id = request.conversation_id or "default"
                if conv_id not in self.conversations:
                    self.conversations[conv_id] = []
                
                messages = self.conversations[conv_id]
                
                # Add user message
                messages.append({"role": "user", "content": request.prompt})
                
                # Query LLM
                response = ollama.chat(model=self.model_name, messages=messages)
                answer = response.message.content
                
                # Add assistant response
                messages.append({"role": "assistant", "content": answer})
                
                # Callback with result
                request.callback(answer)
                
                self.request_queue.task_done()
                
            except queue.Empty:
                continue
            except Exception as e:
                print(f"[GCC] Error: {e}")
    
    def ask(self, prompt: str, callback: Callable[[str], None], 
            conversation_id: Optional[str] = None):
        """
        Ask LLM a question (non-blocking)
        
        Args:
            prompt: Question to ask
            callback: Function to call with answer
            conversation_id: Optional conversation ID for context
        """
        request = GCCRequest(
            prompt=prompt,
            callback=callback,
            conversation_id=conversation_id
        )
        self.request_queue.put(request)
    
    def clear_conversation(self, conversation_id: str = "default"):
        """Clear conversation history"""
        if conversation_id in self.conversations:
            del self.conversations[conversation_id]