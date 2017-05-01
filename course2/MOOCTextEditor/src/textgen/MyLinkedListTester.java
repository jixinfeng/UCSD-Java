/**
 * 
 */
package textgen;

import static org.junit.Assert.*;

import java.util.LinkedList;

import org.junit.Before;
import org.junit.Test;

/**
 * @author UC San Diego MOOC team
 *
 */
public class MyLinkedListTester {

	private static final int LONG_LIST_LENGTH =10; 

	MyLinkedList<String> shortList;
	MyLinkedList<Integer> emptyList;
	MyLinkedList<Integer> longerList;
	MyLinkedList<Integer> list1;
	MyLinkedList<Integer> listTestAddEnd;
	MyLinkedList<Integer> listTestSize;
	
	/**
	 * @throws java.lang.Exception
	 */
	@Before
	public void setUp() throws Exception {
		// Feel free to use these lists, or add your own
	    shortList = new MyLinkedList<String>();
		shortList.add("A");
		shortList.add("B");
		emptyList = new MyLinkedList<Integer>();
		longerList = new MyLinkedList<Integer>();
		for (int i = 0; i < LONG_LIST_LENGTH; i++)
		{
			longerList.add(i);
		}
		list1 = new MyLinkedList<Integer>();
		list1.add(65);
		list1.add(21);
		list1.add(42);
		
		listTestAddEnd = new MyLinkedList<Integer>();
		listTestSize = new MyLinkedList<Integer>();
	}

	
	/** Test if the get method is working correctly.
	 */
	/*You should not need to add much to this method.
	 * We provide it as an example of a thorough test. */
	@Test
	public void testGet()
	{
		//test empty list, get should throw an exception
		try {
			emptyList.get(0);
			fail("Check out of bounds");
		}
		catch (IndexOutOfBoundsException e) {
			
		}
		
		// test short list, first contents, then out of bounds
		assertEquals("Check first", "A", shortList.get(0));
		assertEquals("Check second", "B", shortList.get(1));
		
		try {
			shortList.get(-1);
			fail("Check out of bounds");
		}
		catch (IndexOutOfBoundsException e) {
		
		}
		try {
			shortList.get(2);
			fail("Check out of bounds");
		}
		catch (IndexOutOfBoundsException e) {
		
		}
		// test longer list contents
		for(int i = 0; i<LONG_LIST_LENGTH; i++ ) {
			assertEquals("Check "+i+ " element", (Integer)i, longerList.get(i));
		}
		
		// test off the end of the longer array
		try {
			longerList.get(-1);
			fail("Check out of bounds");
		}
		catch (IndexOutOfBoundsException e) {
		
		}
		try {
			longerList.get(LONG_LIST_LENGTH);
			fail("Check out of bounds");
		}
		catch (IndexOutOfBoundsException e) {
		}
		
	}
	
	
	/** Test removing an element from the list.
	 * We've included the example from the concept challenge.
	 * You will want to add more tests.  */
	@Test
	public void testRemove()
	{
		int a = list1.remove(0);
		assertEquals("Remove: check a is correct ", 65, a);
		assertEquals("Remove: check element 0 is correct ", (Integer)21, list1.get(0));
		assertEquals("Remove: check size is correct ", 2, list1.size());
		
		// TODO: Add more tests here
		try {
			list1.remove(10);
			fail("Check out of bounds");
		}
		catch (IndexOutOfBoundsException e) {
		}
		
		try {
			list1.remove(-1);
			fail("Check out of bounds");
		}
		catch (IndexOutOfBoundsException e) {
		}
		
		int b = list1.remove(1);
		assertEquals("Remove: check a is correct ", 42, b);
		assertEquals("Remove: check element 0 is correct ", (Integer)21, list1.get(0));
		assertEquals("Remove: check size is correct ", 1, list1.size());
		
		int c = list1.remove(0);
		assertEquals("Remove: check a is correct ", 21, c);
		assertEquals("Remove: check size is correct ", 0, list1.size());
	}
	
	/** Test adding an element into the end of the list, specifically
	 *  public boolean add(E element)
	 * */
	@Test
	public void testAddEnd()
	{
        // TODO: implement this test
		try {
			listTestAddEnd.add(null);
			fail("Check null pointer");
		} catch (NullPointerException e) {}
		assertEquals("AddEnd: check list is empty ", 0, listTestAddEnd.size());
		listTestAddEnd.add(0);
		assertEquals("AddEnd: check list size grows correctly ", 1, listTestAddEnd.size());
		assertEquals("AddEnd: check list adds correct element to the end ", (Integer)0, listTestAddEnd.get(0));
	}

	
	/** Test the size of the list */
	@Test
	public void testSize()
	{
		// TODO: implement this test
		for (int i = 0; i < 5; i++)
		{
			assertEquals("Size: check list size ", i, listTestSize.size());
			listTestSize.add(i);
		}
		assertEquals("Size: check list size ", 5, listTestSize.size());
	}

	
	
	/** Test adding an element into the list at a specified index,
	 * specifically:
	 * public void add(int index, E element)
	 * */
	@Test
	public void testAddAtIndex()
	{
        // TODO: implement this test
		try {
			longerList.add(100, 1);
			fail("Check out of bounds");
		} catch (IndexOutOfBoundsException e) {}
		
		try {
			longerList.add(-1, 1);
			fail("Check out of bounds");
		} catch (IndexOutOfBoundsException e) {}
		
		try {
			longerList.add(3, null);
			fail("Check null pointer");
		} catch (NullPointerException e) {}
		
		
		longerList.add(longerList.size(), 100);
		assertEquals("AddAtIndex: check value added", (Integer)100, longerList.get(longerList.size() - 1));
		longerList.add(5, 100);
		assertEquals("AddAtIndex: check value added", (Integer)100, longerList.get(5));
	}
	
	/** Test setting an element in the list */
	@Test
	public void testSet()
	{
	    // TODO: implement this test
		try {
			longerList.set(100, 1);
			fail("Check out of bounds");
		} catch (IndexOutOfBoundsException e) {}
		int a = longerList.set(9, 100);
		assertEquals("Set: check modified data", (Integer)100, longerList.get(9));
		assertEquals("Set: check modified data", 9, a);
	}
	
	
	// TODO: Optionally add more test methods.
	@Test
	public void testCombined()
	{
		longerList.add(10, 100);
		assertEquals("Size: check list size ", 11, longerList.size());
		assertEquals("Size: check list size ", (Integer)100, longerList.get(longerList.size() - 1));
		
		int a = longerList.remove(9);
		assertEquals("Size: check list size ", (Integer)100, longerList.get(longerList.size() - 1));
		assertEquals("Size: check list size ", 9, a);
		
		longerList.add(9, 200);
		assertEquals("Size: check list size ", (Integer)200, longerList.get(9));
		assertEquals("Size: check list size ", 11, longerList.size());
	}
}
