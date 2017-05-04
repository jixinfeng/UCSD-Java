package textgen;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;
import java.util.ListIterator;
import java.util.Random;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

/** 
 * An implementation of the MTG interface that uses a list of lists.
 * @author UC San Diego Intermediate Programming MOOC team 
 */
public class MarkovTextGeneratorLoL implements MarkovTextGenerator {

	// The list of words with their next words
	private List<ListNode> wordList; 
	
	// The starting "word"
	private String starter;
	
	// The random number generator
	private Random rnGenerator;
	
	public MarkovTextGeneratorLoL(Random generator)
	{
		wordList = new LinkedList<ListNode>();
		starter = "";
		rnGenerator = generator;
	}
	
	
	/** Train the generator by adding the sourceText */
	@Override
	public void train(String sourceText)
	{
		// TODO: Implement this method
		if (sourceText.length() == 0) 
		{
			System.out.println("No input string!");
			return;
		} 
		else 
		{
			String[] wordsFromSource = sourceText.split("\\s+");
			int wordsLength = wordsFromSource.length;
	
			starter = wordsFromSource[0];
			String prevWord = starter; 
			for (int i = 1; i < wordsLength; i++)
			{
				String currWord = wordsFromSource[i];
				findAndAppend(prevWord, currWord);
				prevWord = currWord; 
			}
			findAndAppend(prevWord, starter);
		}
	}
	
	/** 
	 * Generate the number of words requested.
	 */
	@Override
	public String generateText(int numWords) {
	    // TODO: Implement this method
		String generatedText = "";
		if (wordList.isEmpty()) 
		{
			System.out.println("Haven't been trained!");
			return generatedText;
		}
		else if (numWords == 0) {return generatedText;}
		
		String currWord = starter;
		List<String> generatedWords = new ArrayList<String>();
		generatedWords.add(currWord);
		while (generatedWords.size() < numWords)
		{
			currWord = pickNextWord(currWord);
			generatedWords.add(currWord);
		}
		generatedText = String.join(" ", generatedWords);
		return generatedText;
	}
	
	
	// Can be helpful for debugging
	@Override
	public String toString()
	{
		String toReturn = "";
		for (ListNode n : wordList)
		{
			toReturn += n.toString();
		}
		return toReturn;
	}
	
	/** Retrain the generator from scratch on the source text */
	@Override
	public void retrain(String sourceText)
	{
		// TODO: Implement this method.
		wordList = new LinkedList<ListNode>();
		starter = "";
		train(sourceText);
	}
	
	// TODO: Add any private helper methods you need here.
	private String pickRandomWord()
	{
		int randomWordIndex;
		String randomWord;
		if (wordList.size() > 0) 
		{
			randomWordIndex = rnGenerator.nextInt(wordList.size());
			randomWord = wordList.get(randomWordIndex).getWord();
		} else
		{
			randomWord = "";
		}
		
		
		return randomWord;
	}
	
	private String pickNextWord(String currWord)
	{
		boolean found = false;
		String nextWord = "";
		for (ListNode currNode: wordList) {
			if (currNode.getWord().equals(currWord))
			{
				nextWord = currNode.getRandomNextWord(rnGenerator);
				if (nextWord != null && !nextWord.isEmpty())
				{
					found = true;
				}
				break;
			}
		}
		if (found) {return nextWord;} 
		else {return pickRandomWord();}
	}
    
    private void findAndAppend(String prevWord, String currWord)
	{
		int nodeToAppendIndex = 0;
		boolean foundWord = false;
		for (int i = 0; i < wordList.size(); i++) 
		{
			ListNode currSearchNode = wordList.get(i);
			String currSearchWord = currSearchNode.getWord();
			if (currSearchWord.equals(prevWord))
			{
				nodeToAppendIndex = i;
				foundWord = true;
				break;
			}
		}
		if (!foundWord)
		{
			nodeToAppendIndex = wordList.size();
			wordList.add(new ListNode(prevWord));
		}
		wordList.get(nodeToAppendIndex).addNextWord(currWord);
	}
	/**
	 * This is a minimal set of tests.  Note that it can be difficult
	 * to test methods/classes with randomized behavior.   
	 * @param args
	 */
	public static void main(String[] args)
	{
		// feed the generator a fixed random value for repeatable behavior
		MarkovTextGeneratorLoL gen = new MarkovTextGeneratorLoL(new Random(42));
		String textString = "a b c d";
		System.out.println(textString);
		gen.train(textString);
		System.out.println(gen);
		System.out.println(gen.generateText(10));
		
		textString = "w x y z";
		System.out.println(textString);
		gen.train(textString);
		System.out.println(gen);
		System.out.println(gen.generateText(20));
		
//		String textString = "Hello.  Hello there.  This is a test.  Hello there.  Hello Bob.  Test again.";
//		System.out.println(textString);
//		gen.train(textString);
//		System.out.println(gen);
//		System.out.println(gen.generateText(20));
//		String textString2 = "You say yes, I say no, "+
//				"You say stop, and I say go, go, go, "+
//				"Oh no. You say goodbye and I say hello, hello, hello, "+
//				"I don't know why you say goodbye, I say hello, hello, hello, "+
//				"I don't know why you say goodbye, I say hello. "+
//				"I say high, you say low, "+
//				"You say why, and I say I don't know. "+
//				"Oh no. "+
//				"You say goodbye and I say hello, hello, hello. "+
//				"I don't know why you say goodbye, I say hello, hello, hello, "+
//				"I don't know why you say goodbye, I say hello. "+
//				"Why, why, why, why, why, why, "+
//				"Do you say goodbye. "+
//				"Oh no. "+
//				"You say goodbye and I say hello, hello, hello. "+
//				"I don't know why you say goodbye, I say hello, hello, hello, "+
//				"I don't know why you say goodbye, I say hello. "+
//				"You say yes, I say no, "+
//				"You say stop and I say go, go, go. "+
//				"Oh, oh no. "+
//				"You say goodbye and I say hello, hello, hello. "+
//				"I don't know why you say goodbye, I say hello, hello, hello, "+
//				"I don't know why you say goodbye, I say hello, hello, hello, "+
//				"I don't know why you say goodbye, I say hello, hello, hello,";
//		System.out.println(textString2);
//		gen.retrain(textString2);
//		System.out.println(gen);
//		System.out.println(gen.generateText(20));
	}

}

/** Links a word to the next words in the list 
 * You should use this class in your implementation. */
class ListNode
{
    // The word that is linking to the next words
	private String word;
	
	// The next words that could follow it
	private List<String> nextWords;
	
	ListNode(String word)
	{
		this.word = word;
		nextWords = new LinkedList<String>();
	}
	
	public String getWord()
	{
		return word;
	}

	public void addNextWord(String nextWord)
	{
		nextWords.add(nextWord);
	}
	
	public String getRandomNextWord(Random generator)
	{
		// TODO: Implement this method
	    // The random number generator should be passed from 
	    // the MarkovTextGeneratorLoL class
		if (nextWords.size() > 0)
		{
			int nextWordIndex = generator.nextInt(nextWords.size());
			return nextWords.get(nextWordIndex);
		} else {return null;}
	}

	public String toString()
	{
		String toReturn = word + ": ";
		for (String s : nextWords) {
			toReturn += s + "->";
		}
		toReturn += "\n";
		return toReturn;
	}
}


