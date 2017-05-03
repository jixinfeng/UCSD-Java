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
		List<String> wordsFromSource = splitWords(sourceText);
//		System.out.println("Print splited words:");
//		System.out.println(wordsFromSource);
		int wordsLength = wordsFromSource.size();
		
		if (wordsLength == 0) {return;}
		else if (wordsLength == 1) 
		{
			wordList.add(new ListNode(wordsFromSource.get(0)));
			return;
		}
		
		int nodeToAppendIndex = 0;
		for (int i = 0; i< wordsLength - 1; i++)
		{
			String newWord = wordsFromSource.get(i); 
			boolean foundWord = false;
			for (int j = 0; j < wordList.size(); j++)
			{
				ListNode currNode = wordList.get(j);
				String currWord = currNode.getWord();
				if (currWord.equals(newWord))
				{
					nodeToAppendIndex = j;
					foundWord = true;
					break;
				}
			}
			if (!foundWord)
			{
				nodeToAppendIndex = wordList.size();
				wordList.add(new ListNode(newWord));
			}
			String nextWord = wordsFromSource.get(i + 1);
			wordList.get(nodeToAppendIndex).addNextWord(nextWord);
		}
	}
	
	/** 
	 * Generate the number of words requested.
	 */
	@Override
	public String generateText(int numWords) {
	    // TODO: Implement this method
		List<String> generatedWords = new ArrayList<String>();
		String currWord = pickRandomWord();
		generatedWords.add(currWord);
		for (int i = 0; i < numWords; i++)
		{
			currWord = pickNextWord(currWord);
			generatedWords.add(currWord);
		}
		return String.join(" ", generatedWords);
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
		train(sourceText);
	}
	
	// TODO: Add any private helper methods you need here.
	private String pickRandomWord()
	{
		int firstWordIndex = rnGenerator.nextInt(wordList.size());
		String firstWord = wordList.get(firstWordIndex).getWord();
		return firstWord;
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
	
    private List<String> splitWords(String text)
    {	
    	String pattern = "[a-zA-z']+";
        ArrayList<String> tokens = new ArrayList<String>();
        Pattern tokSplitter = Pattern.compile(pattern);
        Matcher m = tokSplitter.matcher(text);
        
        while (m.find()) {
            tokens.add(m.group().toLowerCase());
        }
        
        return tokens;
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
		String textString = "Hello.  Hello there.  This is a test.  Hello there.  Hello Bob.  Test again.";
		System.out.println(textString);
		gen.train(textString);
		System.out.println(gen);
		System.out.println(gen.generateText(20));
		String textString2 = "You say yes, I say no, "+
				"You say stop, and I say go, go, go, "+
				"Oh no. You say goodbye and I say hello, hello, hello, "+
				"I don't know why you say goodbye, I say hello, hello, hello, "+
				"I don't know why you say goodbye, I say hello. "+
				"I say high, you say low, "+
				"You say why, and I say I don't know. "+
				"Oh no. "+
				"You say goodbye and I say hello, hello, hello. "+
				"I don't know why you say goodbye, I say hello, hello, hello, "+
				"I don't know why you say goodbye, I say hello. "+
				"Why, why, why, why, why, why, "+
				"Do you say goodbye. "+
				"Oh no. "+
				"You say goodbye and I say hello, hello, hello. "+
				"I don't know why you say goodbye, I say hello, hello, hello, "+
				"I don't know why you say goodbye, I say hello. "+
				"You say yes, I say no, "+
				"You say stop and I say go, go, go. "+
				"Oh, oh no. "+
				"You say goodbye and I say hello, hello, hello. "+
				"I don't know why you say goodbye, I say hello, hello, hello, "+
				"I don't know why you say goodbye, I say hello, hello, hello, "+
				"I don't know why you say goodbye, I say hello, hello, hello,";
		System.out.println(textString2);
		gen.retrain(textString2);
		System.out.println(gen);
		System.out.println(gen.generateText(20));
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


