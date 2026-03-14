#!/usr/bin/env python3
"""
Grace Model Fine-tuning with Unsloth
=====================================

Uses collected feedback to fine-tune Grace's model with DPO
(Direct Preference Optimization)

Optimized for Jetson Orin Nano with 4B models
"""

import torch
from pathlib import Path
import json
from datetime import datetime

try:
    from unsloth import FastLanguageModel
    from unsloth import is_bfloat16_supported
    from trl import DPOTrainer, DPOConfig
    from datasets import Dataset
    UNSLOTH_AVAILABLE = True
except ImportError:
    UNSLOTH_AVAILABLE = False
    print("⚠️  Unsloth not installed!")
    print("Install with: pip install unsloth trl datasets")

from grace_rlhf import get_rlhf


class GraceTrainer:
    """
    Fine-tune Grace using collected feedback
    """
    
    def __init__(
        self,
        base_model: str = "unsloth/mistral-7b-bnb-4bit",
        max_seq_length: int = 2048,
        load_in_4bit: bool = True
    ):
        """
        Initialize trainer
        
        Args:
            base_model: Base model to fine-tune
            max_seq_length: Maximum sequence length
            load_in_4bit: Use 4-bit quantization (for memory efficiency)
        """
        if not UNSLOTH_AVAILABLE:
            raise ImportError("Unsloth not installed")
        
        self.base_model = base_model
        self.max_seq_length = max_seq_length
        self.load_in_4bit = load_in_4bit
        self.rlhf = get_rlhf()
        
        print("=" * 80)
        print("GRACE MODEL TRAINER")
        print("=" * 80)
        print(f"Base model: {base_model}")
        print(f"Max sequence length: {max_seq_length}")
        print(f"4-bit quantization: {load_in_4bit}")
        print()
    
    def prepare_dataset(self):
        """
        Prepare dataset from collected feedback
        
        Returns:
            Hugging Face Dataset ready for DPO training
        """
        print("📦 Preparing dataset...")
        
        # Get training pairs
        pairs = self.rlhf.get_training_data()
        
        if len(pairs) < 10:
            raise ValueError(f"Not enough training data! Need at least 10 pairs, got {len(pairs)}")
        
        print(f"   Found {len(pairs)} training pairs")
        
        # Convert to dataset format
        dataset_dict = {
            'prompt': [],
            'chosen': [],
            'rejected': []
        }
        
        for pair in pairs:
            dataset_dict['prompt'].append(pair['prompt'])
            dataset_dict['chosen'].append(pair['chosen'])
            dataset_dict['rejected'].append(pair['rejected'])
        
        # Create Hugging Face dataset
        dataset = Dataset.from_dict(dataset_dict)
        
        # Split into train/validation
        split = dataset.train_test_split(test_size=0.1, seed=42)
        
        print(f"   Train set: {len(split['train'])} pairs")
        print(f"   Val set: {len(split['test'])} pairs")
        print()
        
        return split['train'], split['test']
    
    def load_model(self):
        """
        Load base model with Unsloth optimizations
        
        Returns:
            model, tokenizer
        """
        print("🔧 Loading model...")
        
        model, tokenizer = FastLanguageModel.from_pretrained(
            model_name=self.base_model,
            max_seq_length=self.max_seq_length,
            dtype=None,  # Auto-detect
            load_in_4bit=self.load_in_4bit,
        )
        
        print("   Model loaded successfully")
        print()
        
        return model, tokenizer
    
    def setup_peft(self, model):
        """
        Setup PEFT (Parameter Efficient Fine-Tuning) with LoRA
        
        Args:
            model: Base model
        
        Returns:
            PEFT model
        """
        print("⚙️  Setting up LoRA...")
        
        model = FastLanguageModel.get_peft_model(
            model,
            r=16,  # LoRA rank
            target_modules=[
                "q_proj", "k_proj", "v_proj", "o_proj",
                "gate_proj", "up_proj", "down_proj"
            ],
            lora_alpha=16,
            lora_dropout=0.0,  # Optimized for Unsloth
            bias="none",
            use_gradient_checkpointing="unsloth",  # Unsloth optimization
            random_state=42,
        )
        
        print("   LoRA configured")
        print()
        
        return model
    
    def train(
        self,
        output_dir: Path = None,
        num_epochs: int = 3,
        batch_size: int = 2,
        learning_rate: float = 5e-5,
    ):
        """
        Fine-tune Grace with DPO
        
        Args:
            output_dir: Where to save fine-tuned model
            num_epochs: Number of training epochs
            batch_size: Batch size (keep small for Jetson)
            learning_rate: Learning rate
        """
        output_dir = output_dir or Path.home() / "AGi" / "models" / f"grace_finetuned_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
        output_dir.mkdir(parents=True, exist_ok=True)
        
        print("🎓 Starting training...")
        print(f"   Output directory: {output_dir}")
        print()
        
        # Prepare data
        train_dataset, val_dataset = self.prepare_dataset()
        
        # Load model
        model, tokenizer = self.load_model()
        
        # Setup LoRA
        model = self.setup_peft(model)
        
        # DPO Configuration
        training_args = DPOConfig(
            output_dir=str(output_dir),
            num_train_epochs=num_epochs,
            per_device_train_batch_size=batch_size,
            per_device_eval_batch_size=batch_size,
            learning_rate=learning_rate,
            logging_steps=10,
            save_steps=100,
            eval_steps=50,
            warmup_steps=50,
            optim="adamw_8bit",  # Memory efficient
            fp16=not is_bfloat16_supported(),
            bf16=is_bfloat16_supported(),
            logging_dir=str(output_dir / "logs"),
            report_to="none",  # Disable wandb
        )
        
        # Create DPO trainer
        trainer = DPOTrainer(
            model=model,
            args=training_args,
            train_dataset=train_dataset,
            eval_dataset=val_dataset,
            tokenizer=tokenizer,
            beta=0.1,  # DPO beta parameter
        )
        
        # Train!
        print("🚀 Training started...")
        print()
        
        trainer.train()
        
        print()
        print("✅ Training complete!")
        print()
        
        # Save model
        print("💾 Saving model...")
        model.save_pretrained(str(output_dir / "final_model"))
        tokenizer.save_pretrained(str(output_dir / "final_model"))
        
        print(f"   Model saved to: {output_dir / 'final_model'}")
        print()
        
        # Save training metadata
        metadata = {
            'training_date': datetime.now().isoformat(),
            'base_model': self.base_model,
            'num_epochs': num_epochs,
            'batch_size': batch_size,
            'learning_rate': learning_rate,
            'train_samples': len(train_dataset),
            'val_samples': len(val_dataset),
            'output_dir': str(output_dir)
        }
        
        with open(output_dir / "training_metadata.json", 'w') as f:
            json.dump(metadata, f, indent=2)
        
        return output_dir
    
    def export_gguf(self, model_path: Path, output_path: Path = None):
        """
        Export fine-tuned model to GGUF format for Ollama
        
        Args:
            model_path: Path to fine-tuned model
            output_path: Where to save GGUF file
        """
        print("📦 Exporting to GGUF format...")
        
        if not output_path:
            output_path = model_path.parent / f"{model_path.name}.gguf"
        
        # Load model
        model, tokenizer = FastLanguageModel.from_pretrained(
            model_name=str(model_path),
            max_seq_length=self.max_seq_length,
            dtype=None,
            load_in_4bit=False,  # Export in full precision
        )
        
        # Export
        model.save_pretrained_gguf(
            str(output_path),
            tokenizer,
            quantization_method="q4_k_m"  # 4-bit quantization
        )
        
        print(f"   GGUF saved to: {output_path}")
        print()
        print("To use in Ollama:")
        print(f"   1. Copy GGUF to ~/.ollama/models/")
        print(f"   2. Create Modelfile")
        print(f"   3. Run: ollama create grace-finetuned -f Modelfile")
        print()
        
        return output_path


def main():
    """
    Main training script
    """
    import argparse
    
    parser = argparse.ArgumentParser(description="Fine-tune Grace with collected feedback")
    parser.add_argument("--base-model", default="unsloth/mistral-7b-bnb-4bit", help="Base model to fine-tune")
    parser.add_argument("--epochs", type=int, default=3, help="Number of epochs")
    parser.add_argument("--batch-size", type=int, default=2, help="Batch size")
    parser.add_argument("--lr", type=float, default=5e-5, help="Learning rate")
    parser.add_argument("--export-gguf", action="store_true", help="Export to GGUF after training")
    
    args = parser.parse_args()
    
    # Check if we have enough data
    rlhf = get_rlhf()
    rlhf.print_stats()
    
    if not rlhf.should_retrain(threshold=10):
        print("❌ Not enough training data collected yet!")
        print("   Need at least 10 preference pairs")
        return
    
    # Initialize trainer
    trainer = GraceTrainer(
        base_model=args.base_model,
        max_seq_length=2048,
        load_in_4bit=True
    )
    
    # Train
    output_dir = trainer.train(
        num_epochs=args.epochs,
        batch_size=args.batch_size,
        learning_rate=args.lr
    )
    
    # Export to GGUF if requested
    if args.export_gguf:
        trainer.export_gguf(output_dir / "final_model")
    
    print("=" * 80)
    print("🎉 TRAINING COMPLETE!")
    print("=" * 80)
    print()
    print("Next steps:")
    print("1. Test the fine-tuned model")
    print("2. If satisfied, deploy to Ollama")
    print("3. Update your cnc.py to use the new model")
    print()


if __name__ == "__main__":
    main()
