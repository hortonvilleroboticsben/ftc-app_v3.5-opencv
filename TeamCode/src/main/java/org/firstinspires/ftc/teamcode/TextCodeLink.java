package org.firstinspires.ftc.teamcode;

import com.sun.tools.javac.code.Attribute;

import java.util.*;

import java.io.*;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;
import java.lang.annotation.Annotation;
import java.lang.annotation.ElementType;
import java.lang.reflect.*;

public class TextCodeLink<T extends Class<?>> {
	T c;
	File folder;
	String name;
	File file;
	InputStream in;
	ArrayList<BoundPair> keys = new ArrayList<BoundPair>();
	Queue<String> fileContent = new LinkedList<String>();

	public TextCodeLink(T c, File folder, String name) {
		this.folder = folder;
		this.c = c;
		this.name = name;
		folder.mkdirs();
		try {
			file = new File(folder, name);
			if(!file.exists()) {
				file.createNewFile();
//				openFile();
				throw new Exception("File "+file.getPath()+" not found, creating and opening...");
			}
			in = new FileInputStream(file);
		}catch(Exception e) {
			e.printStackTrace();
		}
		linkMethods();
	}


	public void linkMethods() {
		for(Method m : c.getMethods()) {
			for(Annotation a : m.getAnnotations()) {
				if(a.annotationType() == TextLink.class) {
					TextLink t = (TextLink) a;
					keys.add(new BoundPair(t.key(), m));
				}
			}
		}
	}

	public void parseFile(){
		String cAdd = "";
		char curr = Character.MIN_VALUE;
		try {
			in = new FileInputStream(file);
			while(in.available() > 0) {
				curr = (char)in.read();
				if(curr == '\r' || curr == '#' || curr == ',') {
					if(curr == '\r')in.read();
					if(!cAdd.equals("") && !(cAdd.indexOf("*") == 0))fileContent.add(cAdd);
					cAdd="";
				} else cAdd+=curr;
			}
			if(!cAdd.equals(""))fileContent.add(cAdd);
			cAdd="";
		} catch (Exception e) {
			e.printStackTrace();
		}
	}

//	public void openFile() {
//		try {
//			if(Desktop.isDesktopSupported()) {
//				if(file.exists())Desktop.getDesktop().open(file);
//			}
//		}catch(Exception e) {
//			e.printStackTrace();
//		}
//	}

	public void checkSyntax() {
		try {
			for(String s : fileContent) {
				boolean isContained = false;
				for(int i = 0; i < keys.size(); i++) {
					BoundPair bP = keys.get(i);
					if(s.equals(bP.key)) {
						isContained = true;
						break;
					}
				}
				if(!isContained) throw new Exception("Token not recognized:"+s);
			}
		}catch(Exception e) {
			e.printStackTrace();
		}
	}

	public void runFile() throws Exception {
		String s = "";
		try {
			while(fileContent.peek() != null) {
				s = fileContent.poll();
				int foundInd = 0;
				for(int i = 0; i < keys.size(); i++) {
					BoundPair bP = keys.get(i);
					if(s.equals(bP.key)) {
						foundInd = i;
						break;
					}
				}
				Method method = keys.get(foundInd).method;
				Object[] args = new Object[method.getParameterTypes().length];
				for(int i = 0; i < args.length; i++) {
					Class<?> curr =	method.getParameterTypes()[i];
					if(curr.equals(Integer.class) || curr.equals(int.class)) {
						args[i] = (int)Integer.parseInt(fileContent.poll());
					}else if (curr.equals(Long.class) || curr.equals(long.class)) {
						args[i] = (long)Long.parseLong(fileContent.poll());
					}else if (curr.equals(Double.class) || curr.equals(double.class)) {
						args[i] = (double) Double.parseDouble(fileContent.poll());
					}else if (curr.equals(String.class)) {
						args[i] = new String(fileContent.poll());
					}else throw new Exception("Method "+method.getName()+" cannot have parameter type "
							+method.getParameterTypes()[i]);
				}
				try {
					method.invoke(null, args);
				}catch (Exception e){
					TestFileReader t = new TestFileReader();
				}
			}
		}catch(Exception e) {
			e.printStackTrace();
			throw new Exception("Token not recognized:"+s);
		}
	}
}

class BoundPair{
	String key;
	Method method;
	public BoundPair(String k, Method m) {
		key = k;
		method = m;
	}

	public String toString() {
		return "["+key+", "+method.getName()+"]";
	}
}


@Target(ElementType.METHOD)
@Retention(RetentionPolicy.RUNTIME)
@interface TextLink{
	String key();
}
