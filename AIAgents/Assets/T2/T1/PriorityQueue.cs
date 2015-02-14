using UnityEngine;
using System.Collections;
using System.Collections.Generic;

public class PriorityQueue<Tkey, Tvalue> {
	List<KeyValuePair<Tkey, Tvalue>> heap;
	IComparer<Tvalue> comparer;

	public PriorityQueue() : this(Comparer<Tvalue>.Default){

	}

	public PriorityQueue(IComparer<Tvalue> comparer) {
/*		if (comparer == null)
			throw "PriorityQueue: Null argument exception";
*/
		heap = new List<KeyValuePair<Tkey, Tvalue>> ();
		this.comparer = comparer;
	}

	public void Enqueue(Tkey key, Tvalue val) {
		Insert (key, val);
	}

	private void Insert(Tkey key, Tvalue value) {
		KeyValuePair<Tkey, Tvalue> val =
			new KeyValuePair<Tkey, Tvalue> (key, value);
		heap.Add (val);

		// heapify after insert
		HeapifyFromEndToBeginning (heap.Count - 1);
	}

	private int HeapifyFromEndToBeginning(int pos){
		if(pos >= heap.Count)
			return -1;

		// heap[i] have children heap[2*i + 1] and heap[2*i + 2] and parent heap[(i-1)/ 2];
		while (pos > 0) {
			int parentPos = Mathf.RoundToInt ((pos - 1) / 2);
			if(comparer.Compare (heap[parentPos].Value, heap[pos].Value) > 0) {
				ExchangeElements(parentPos, pos);
				pos = parentPos;
			}
			else
				break;
		}
		return pos;
	}

	private void ExchangeElements(int pos1, int pos2) {
		KeyValuePair<Tkey, Tvalue> val = heap [pos1];
		heap [pos1] = heap [pos2];
		heap [pos2] = val;
	}

	public int Count() {
		return heap.Count;
	}
	/*
	public KeyValuePair<Tkey, Tvalue> Dequeue() {
		if (heap.Count != 0) {
			KeyValuePair<Tkey, Tvalue> result = heap[0];
			DeleteRoot();
			return result;
		}
		else
			throw new System.InvalidOperationException("Priority queue is empty");
	}
	*/

	public Tkey Dequeue() {
		if (heap.Count != 0) {
			KeyValuePair<Tkey, Tvalue> result = heap[0];
			DeleteRoot();
			return result.Key;
		}
		else
			throw new System.InvalidOperationException("Priority queue is empty");
	}

	private void DeleteRoot()
	{
		if (heap.Count <= 1)
		{
			heap.Clear();
			return;
		}
		
		heap[0] = heap[heap.Count - 1];
		heap.RemoveAt(heap.Count - 1);
		
		// heapify
		HeapifyFromBeginningToEnd(0);
	}

	private void HeapifyFromBeginningToEnd(int pos)
	{
		if (pos >= heap.Count) return;
		
		// heap[i] have children heap[2*i + 1] and heap[2*i + 2] and parent heap[(i-1)/ 2];
		
		while (true)
		{
			// on each iteration exchange element with its smallest child
			int smallest = pos;
			int left = 2 * pos + 1;
			int right = 2 * pos + 2;
			if (left < heap.Count &&
			    comparer.Compare(heap[smallest].Value, heap[left].Value) > 0)
				smallest = left;
			if (right < heap.Count &&
			    comparer.Compare(heap[smallest].Value, heap[right].Value) > 0)
				smallest = right;
			
			if (smallest != pos)
			{
				ExchangeElements(smallest, pos);
				pos = smallest;
			}
			else break;
		}
	}

}
