<?xml version="1.0" encoding="UTF-8"?>
<xsl:stylesheet version="1.0"
xmlns:xsl="http://www.w3.org/1999/XSL/Transform">
<xsl:output
method="text"/>
<xsl:variable name='newline'><xsl:text>
</xsl:text></xsl:variable>
<xsl:template match="/">
      <xsl:for-each select="testsuites/testsuite">
        <xsl:for-each select="testcase[@status='Failed']">
          <xsl:value-of select="@name"/><xsl:text>&#xa;</xsl:text>
        </xsl:for-each>
      </xsl:for-each>
</xsl:template>
</xsl:stylesheet>

