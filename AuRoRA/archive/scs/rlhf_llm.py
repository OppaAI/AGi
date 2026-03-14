"""
Grace RLHF (Reinforcement Learning from Human Feedback) System
===============================================================

Integrates with CNS Bridge to learn from:
- 👍 Pet interactions (positive feedback)
- 👎 Red X button (negative feedback)

Strategy: Lightweight RLHF optimized for Jetson Orin Nano
- Collects feedback pairs
- Periodic fine-tuning with Unsloth
- Memory-efficient training
"""

import json
from pathlib import Path
from datetime import datetime
from typing import Literal, Optional, Dict, List
import threading
from collections import deque


class GraceRLHF:
    """
    Reinforcement Learning from Human Feedback for Grace
    
    Features:
    - Feedback collection (pet = positive, red X = negative)
    - Preference pair generation
    - Training data preparation for Unsloth
    - Automatic model updates
    """
    
    def __init__(self, data_dir: Path = None):
        """
        Initialize RLHF system
        
        Args:
            data_dir: Directory to store feedback data
        """
        self.data_dir = data_dir or Path.home() / "AGi" / "rlhf_data"
        self.data_dir.mkdir(parents=True, exist_ok=True)
        
        # File paths
        self.feedback_file = self.data_dir / "feedback.jsonl"
        self.training_pairs_file = self.data_dir / "training_pairs.jsonl"
        self.stats_file = self.data_dir / "stats.json"
        
        # In-memory buffers
        self.feedback_buffer = deque(maxlen=1000)  # Recent feedback
        self.current_interaction = None  # Track current response
        
        # Statistics
        self.stats = self._load_stats()
        
        # Thread safety
        self.lock = threading.Lock()
        
        print("✅ Grace RLHF System initialized")
        print(f"   Data directory: {self.data_dir}")
        print(f"   Total feedback collected: {self.stats['total_feedback']}")
    
    def start_interaction(self, prompt: str, response_id: str = None):
        """
        Start tracking a new interaction
        
        Args:
            prompt: User's message
            response_id: Unique identifier for this response
        """
        with self.lock:
            self.current_interaction = {
                'id': response_id or self._generate_id(),
                'timestamp': datetime.now().isoformat(),
                'prompt': prompt,
                'response': None,
                'feedback': None
            }
    
    def set_response(self, response: str):
        """
        Set the LLM response for current interaction
        
        Args:
            response: Grace's response text
        """
        with self.lock:
            if self.current_interaction:
                self.current_interaction['response'] = response
    
    def record_feedback(
        self, 
        feedback_type: Literal['positive', 'negative'],
        prompt: str = None,
        response: str = None
    ) -> bool:
        """
        Record user feedback (pet or red X)
        
        Args:
            feedback_type: 'positive' (pet) or 'negative' (red X)
            prompt: User's message (if not using current interaction)
            response: Grace's response (if not using current interaction)
        
        Returns:
            True if feedback recorded successfully
        """
        with self.lock:
            # Use provided data or current interaction
            if prompt and response:
                interaction = {
                    'id': self._generate_id(),
                    'timestamp': datetime.now().isoformat(),
                    'prompt': prompt,
                    'response': response,
                    'feedback': feedback_type
                }
            elif self.current_interaction and self.current_interaction['response']:
                interaction = self.current_interaction.copy()
                interaction['feedback'] = feedback_type
            else:
                print("⚠️  No interaction to record feedback for")
                return False
            
            # Save to buffer and file
            self.feedback_buffer.append(interaction)
            self._append_to_file(self.feedback_file, interaction)
            
            # Update stats
            self.stats['total_feedback'] += 1
            self.stats[f'{feedback_type}_count'] += 1
            self._save_stats()
            
            # Generate training pairs if we have enough data
            if len(self.feedback_buffer) >= 10:
                self._generate_training_pairs()
            
            emoji = "👍" if feedback_type == 'positive' else "👎"
            print(f"{emoji} Feedback recorded: {feedback_type}")
            print(f"   Prompt: {interaction['prompt'][:50]}...")
            print(f"   Response: {interaction['response'][:50]}...")
            
            return True
    
    def _generate_training_pairs(self):
        """
        Generate preference pairs for DPO/RLHF training
        
        Creates pairs of (prompt, chosen_response, rejected_response)
        where chosen has positive feedback, rejected has negative
        """
        positive_examples = [
            fb for fb in self.feedback_buffer 
            if fb['feedback'] == 'positive'
        ]
        
        negative_examples = [
            fb for fb in self.feedback_buffer 
            if fb['feedback'] == 'negative'
        ]
        
        # Match positive and negative responses for similar prompts
        pairs = []
        
        for pos in positive_examples:
            # Find similar prompts in negative examples
            for neg in negative_examples:
                similarity = self._prompt_similarity(pos['prompt'], neg['prompt'])
                
                if similarity > 0.5:  # Similar enough
                    pair = {
                        'prompt': pos['prompt'],
                        'chosen': pos['response'],
                        'rejected': neg['response'],
                        'positive_id': pos['id'],
                        'negative_id': neg['id'],
                        'similarity': similarity,
                        'created_at': datetime.now().isoformat()
                    }
                    pairs.append(pair)
                    self._append_to_file(self.training_pairs_file, pair)
        
        if pairs:
            print(f"✨ Generated {len(pairs)} training pairs")
            self.stats['training_pairs'] = self.stats.get('training_pairs', 0) + len(pairs)
            self._save_stats()
    
    def _prompt_similarity(self, prompt1: str, prompt2: str) -> float:
        """
        Simple similarity metric between prompts
        (In production, use embeddings)
        """
        words1 = set(prompt1.lower().split())
        words2 = set(prompt2.lower().split())
        
        if not words1 or not words2:
            return 0.0
        
        intersection = len(words1 & words2)
        union = len(words1 | words2)
        
        return intersection / union if union > 0 else 0.0
    
    def get_training_data(self) -> List[Dict]:
        """
        Get all training pairs for model fine-tuning
        
        Returns:
            List of preference pairs in Unsloth DPO format
        """
        pairs = []
        
        if self.training_pairs_file.exists():
            with open(self.training_pairs_file, 'r') as f:
                for line in f:
                    if line.strip():
                        pairs.append(json.loads(line))
        
        return pairs
    
    def export_for_unsloth(self, output_file: Path = None) -> Path:
        """
        Export training data in Unsloth-compatible format
        
        Args:
            output_file: Where to save the training data
        
        Returns:
            Path to exported file
        """
        output_file = output_file or self.data_dir / "unsloth_training_data.jsonl"
        
        pairs = self.get_training_data()
        
        # Convert to Unsloth DPO format
        unsloth_data = []
        for pair in pairs:
            unsloth_data.append({
                'prompt': pair['prompt'],
                'chosen': pair['chosen'],
                'rejected': pair['rejected']
            })
        
        # Save in JSONL format
        with open(output_file, 'w') as f:
            for item in unsloth_data:
                f.write(json.dumps(item) + '\n')
        
        print(f"📦 Exported {len(unsloth_data)} training pairs to {output_file}")
        return output_file
    
    def get_feedback_context(self, max_examples: int = 5) -> str:
        """
        Get recent feedback as context for prompt engineering
        (Alternative to fine-tuning - adds to system prompt)
        
        Args:
            max_examples: Maximum number of examples to include
        
        Returns:
            Formatted string with feedback examples
        """
        recent_positive = [
            fb for fb in list(self.feedback_buffer)[-20:]
            if fb['feedback'] == 'positive'
        ][:max_examples]
        
        recent_negative = [
            fb for fb in list(self.feedback_buffer)[-20:]
            if fb['feedback'] == 'negative'
        ][:max_examples]
        
        context = "User preferences (recent feedback):\n\n"
        
        if recent_positive:
            context += "✅ Liked responses:\n"
            for fb in recent_positive:
                context += f"- User: {fb['prompt'][:40]}...\n"
                context += f"  Grace: {fb['response'][:60]}...\n\n"
        
        if recent_negative:
            context += "❌ Disliked responses:\n"
            for fb in recent_negative:
                context += f"- User: {fb['prompt'][:40]}...\n"
                context += f"  Grace (avoid this): {fb['response'][:60]}...\n\n"
        
        return context if (recent_positive or recent_negative) else ""
    
    def should_retrain(self, threshold: int = 50) -> bool:
        """
        Check if we have enough feedback to retrain
        
        Args:
            threshold: Minimum number of training pairs needed
        
        Returns:
            True if retraining is recommended
        """
        pairs = self.get_training_data()
        return len(pairs) >= threshold
    
    def _generate_id(self) -> str:
        """Generate unique ID for interaction"""
        return f"{datetime.now().strftime('%Y%m%d_%H%M%S_%f')}"
    
    def _append_to_file(self, filepath: Path, data: dict):
        """Append JSON line to file"""
        with open(filepath, 'a') as f:
            f.write(json.dumps(data) + '\n')
    
    def _load_stats(self) -> dict:
        """Load statistics from file"""
        if self.stats_file.exists():
            with open(self.stats_file, 'r') as f:
                return json.load(f)
        
        return {
            'total_feedback': 0,
            'positive_count': 0,
            'negative_count': 0,
            'training_pairs': 0
        }
    
    def _save_stats(self):
        """Save statistics to file"""
        with open(self.stats_file, 'w') as f:
            json.dump(self.stats, f, indent=2)
    
    def get_stats(self) -> dict:
        """Get current statistics"""
        return {
            **self.stats,
            'buffer_size': len(self.feedback_buffer),
            'positive_ratio': (
                self.stats['positive_count'] / self.stats['total_feedback']
                if self.stats['total_feedback'] > 0 else 0
            )
        }
    
    def print_stats(self):
        """Print formatted statistics"""
        stats = self.get_stats()
        
        print("\n" + "=" * 60)
        print("📊 GRACE RLHF STATISTICS")
        print("=" * 60)
        print(f"Total feedback: {stats['total_feedback']}")
        print(f"  👍 Positive: {stats['positive_count']}")
        print(f"  👎 Negative: {stats['negative_count']}")
        print(f"  Ratio: {stats['positive_ratio']:.1%} positive")
        print(f"\nTraining pairs generated: {stats['training_pairs']}")
        print(f"Current buffer: {stats['buffer_size']} interactions")
        print(f"\nReady to retrain: {'✅ Yes' if self.should_retrain() else '❌ Not yet'}")
        print("=" * 60 + "\n")


# Singleton instance
_rlhf_instance = None

def get_rlhf() -> GraceRLHF:
    """Get or create RLHF singleton instance"""
    global _rlhf_instance
    if _rlhf_instance is None:
        _rlhf_instance = GraceRLHF()
    return _rlhf_instance

if __name__ == "__main__":
    rlhf = get_rlhf()
    rlhf.print_stats()
