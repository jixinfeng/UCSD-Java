package textgen;

import java.util.AbstractList;


/** A class that implements a doubly linked list
 * 
 * @author UC San Diego Intermediate Programming MOOC team
 *
 * @param <E> The type of the elements stored in the list
 */
public class MyLinkedList<E> extends AbstractList<E> {
	LLNode<E> head;
	LLNode<E> tail;
	int size;

	/** Create a new empty LinkedList */
	public MyLinkedList() {
		// TODO: Implement this method
		head = new LLNode<E>(null);
		tail = new LLNode<E>(null);
		head.next = tail;
		tail.prev = head;
		size = 0;
	}

	/**
	 * Appends an element to the end of the list
	 * @param element The element to add
	 */
	public boolean add(E element) 
	{
		// TODO: Implement this method
		if (element == null){throw new NullPointerException();}
		LLNode<E> ins = new LLNode<E>(element);
		ins.prev = tail.prev;
		ins.next = tail;
		tail.prev.next = ins;
		tail.prev = ins;
		size++;
		return true;
	}

	/** Get the element at position index 
	 * @throws IndexOutOfBoundsException if the index is out of bounds. */
	public E get(int index) 
	{
		// TODO: Implement this method.
		if (index < 0 || index >= size) 
		{
			throw new IndexOutOfBoundsException();
		}
		LLNode<E> p = head;
		for (int i = 0; i < index; i++) 
		{
			p = p.next;
		}
		return p.next.data;
	}

	/**
	 * Add an element to the list at the specified index
	 * @param The index where the element should be added
	 * @param element The element to add
	 */
	public void add(int index, E element ) 
	{
		// TODO: Implement this method
		if (index < 0 || index > size) 
		{
			throw new IndexOutOfBoundsException();
		} else if (index == size) 
		{
			this.add(element);
		} else
		{
			if (element == null){throw new NullPointerException();}
			LLNode<E> p = head;
			
			for (int i = 0; i < index; i++) {p = p.next;}
			
			LLNode<E> ins = new LLNode<E>(element);
			ins.prev = p;
			ins.next = p.next;
			p.next.prev = ins;
			p.next = ins;
			size++;
		}
	}


	/** Return the size of the list */
	public int size() 
	{
		// TODO: Implement this method
		return size;
	}

	/** Remove a node at the specified index and return its data element.
	 * @param index The index of the element to remove
	 * @return The data element removed
	 * @throws IndexOutOfBoundsException If index is outside the bounds of the list
	 * 
	 */
	public E remove(int index) 
	{
		// TODO: Implement this method
		if (index < 0 || index >= size) {throw new IndexOutOfBoundsException();}
		
		LLNode<E> p = head;
		for (int i = 0; i < index; i++) {p = p.next;}
		
		LLNode<E> removed = p.next;
		p.next = p.next.next;
		p.next.prev = p;
		size--;
		return removed.data;
	}

	/**
	 * Set an index position in the list to a new element
	 * @param index The index of the element to change
	 * @param element The new element
	 * @return The element that was replaced
	 * @throws IndexOutOfBoundsException if the index is out of bounds.
	 */
	public E set(int index, E element) 
	{
		// TODO: Implement this method
		if (index < 0 || index >= size) {throw new IndexOutOfBoundsException();}
		
		if (element == null){throw new NullPointerException();}
		
		LLNode<E> p = head;
		for (int i = 0; i < index; i++) {p = p.next;}
		
		LLNode<E> replaced = p.next;
		LLNode<E> ins = new LLNode<E>(element);
		
		ins.prev = replaced.prev;
		ins.next = replaced.next;
		
		replaced.prev.next = ins;
		replaced.next.prev = ins;
		
		return replaced.data;
	}   
}

class LLNode<E> 
{
	LLNode<E> prev;
	LLNode<E> next;
	E data;

	// TODO: Add any other methods you think are useful here
	// E.g. you might want to add another constructor
	public LLNode(E e) 
	{
		this.data = e;
		this.prev = null;
		this.next = null;
	}

}
